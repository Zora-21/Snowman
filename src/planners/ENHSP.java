package planners;

/*
 * Copyright (C) 2015-2017, Enrico Scala, contact: enricos83@gmail.com
 * Modified for Snowman Heuristic Extension, 2026
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

import com.hstairs.ppmajal.conditions.BoolPredicate;
import com.hstairs.ppmajal.conditions.PDDLObject;
import com.hstairs.ppmajal.domain.PDDLDomain;
import com.hstairs.ppmajal.expressions.NumFluent;
import com.hstairs.ppmajal.pddl.heuristics.PDDLHeuristic;
import com.hstairs.ppmajal.problem.PDDLObjects;
import com.hstairs.ppmajal.problem.PDDLProblem;
import com.hstairs.ppmajal.problem.PDDLSearchEngine;
import com.hstairs.ppmajal.problem.PDDLState;
import com.hstairs.ppmajal.problem.State;
import com.hstairs.ppmajal.search.SearchEngine;
import com.hstairs.ppmajal.search.SearchHeuristic;
import com.hstairs.ppmajal.transition.TransitionGround;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.apache.commons.lang3.tuple.Pair;

import models.Ball;
import models.SnowmanConfigurator;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.math.BigDecimal;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ENHSP {

    // ==================================================================================
    // Standard ENHSP fields
    // ==================================================================================

    private String domainFile;
    private String problemFile;
    private String searchEngineString;
    private String hw;
    private String heuristic = "aibr";
    private boolean saving_json = false;
    private String deltaExecution;
    private float depthLimit;
    private String savePlan;
    private String tieBreaking;
    private String planner;
    private String deltaHeuristic;
    private String deltaPlanning;
    private String deltaValidation;
    private boolean helpfulActionsPruning;
    private PDDLProblem problem;
    private boolean pddlPlus;
    private PDDLDomain domain;
    private PDDLProblem heuristicProblem;
    private long overallStart;
    private boolean anyTime;
    private long timeOut;
    private boolean aibrPreprocessing;
    private SearchHeuristic h;
    private long overallPlanningTime;
    private float endGValue;
    private boolean helpfulTransitions;
    private int planLength;
    private String redundantConstraints;
    private String groundingType;
    private boolean stopAfterGrounding;
    private boolean printEvents;
    private boolean sdac;
    private boolean onlyPlan;
    private boolean ignoreMetric;

    // ==================================================================================
    // Snowman-specific fields
    // ==================================================================================

    /** Names of ball objects extracted from the PDDL problem. */
    private List<String> ballNames = new ArrayList<>();

    /** Number of snowmen to build (= ballNames.size() / 3). */
    private int targetSnowmen = 0;

    /** Grounded boolean fluents, copied from the PPMAJAL problem instance. */
    private HashSet<BoolPredicate> groundedBooleanFluents;

    /**
     * All-Pairs Shortest Path over the maze graph (character movement, cost =
     * hops).
     */
    private int[][] distanceMatrix;

    /** dirMatrix[u][v] = direction label of the edge u→v (null if not adjacent). */
    private int[][] dirMatrix;

    /**
     * pushDistMatrix[a][b] = minimum number of ball PUSHES to move a ball
     * from location a to location b.
     *
     * Computed via 0-1 BFS where
     * - character walk edges have cost 0 (move_character costs nothing in the
     * metric)
     * - ball push edges have cost 1 (move_ball costs 1 in the metric)
     *
     * The previous implementation used a uniform BFS (cost 1 for every edge),
     * which inflated all estimates and made the heuristic inadmissible.
     */
    private int[][] pushDistMatrix;

    /**
     * isDeadEndNode[i] = true if node i has degree ≤ 1 in the maze graph.
     * A ball pushed into a dead-end cannot be pushed out again.
     */
    private boolean[] isDeadEndNode;
    /**
     * isCornerNode[i] = true if node i has degree = 2 with orthogonal directions.
     * A ball pushed into a corner can never leave (Sokoban freeze rule).
     */
    private boolean[] isCornerNode;

    /**
     * Adjacency lists built from dirMatrix for O(deg) iteration.
     * adjList[u] = array of neighbors v where dirMatrix[u][v] != -1
     * adjDir[u] = corresponding direction labels (parallel to adjList)
     * oppAdj[u] = for each neighbor v in adjList[u], the single neighbor w
     * of u in the OPPOSITE direction (or -1 if none).
     * Used for push-feasibility: to push u→v, char must be at oppAdj[u][i].
     */
    private int[][] adjList;
    private int[][] adjDir;
    private int[][] oppAdj;

    // ==================================================================================
    // Location index helpers
    // ==================================================================================

    private List<String> locationNames = new ArrayList<>();
    private Map<String, Integer> locationIndexMap = new HashMap<>();

    private int getLocationIndex(String name) {
        Integer idx = locationIndexMap.get(name);
        return idx != null ? idx : -1;
    }

    private int getLocIdIgnoreCase(String token) {
        for (Map.Entry<String, Integer> e : locationIndexMap.entrySet())
            if (e.getKey().equalsIgnoreCase(token))
                return e.getValue();
        return -1;
    }

    private String getBallNameIgnoreCase(String token) {
        for (String b : ballNames)
            if (b.equalsIgnoreCase(token))
                return b;
        return null;
    }

    // ==================================================================================
    // Constructor
    // ==================================================================================

    public ENHSP(boolean copyProblem) {
    }

    public int getPlanLength() {
        return planLength;
    }

    // ==================================================================================
    // Domain / problem parsing (unchanged from original ENHSP)
    // ==================================================================================

    public Pair<PDDLDomain, PDDLProblem> parseDomainProblem(String domainFile,
            String problemFile, String delta, PrintStream out) {
        try {
            final PDDLDomain localDomain = new PDDLDomain(domainFile);
            pddlPlus = !localDomain.getProcessesSchema().isEmpty()
                    || !localDomain.getEventsSchema().isEmpty();
            out.println("Domain parsed");
            final PDDLProblem localProblem = new PDDLProblem(problemFile,
                    localDomain.getConstants(), localDomain.getTypes(), localDomain,
                    out, groundingType, sdac, ignoreMetric,
                    new BigDecimal(delta), new BigDecimal(delta));
            if (!localDomain.getProcessesSchema().isEmpty())
                localProblem.setDeltaTimeVariable(delta);
            out.println("Problem parsed");
            out.println("Grounding..");
            localProblem.prepareForSearch(aibrPreprocessing, stopAfterGrounding);

            this.groundedBooleanFluents = new HashSet<>();
            try {
                for (Object f : localProblem.getAllFluents()) {
                    if (f instanceof BoolPredicate)
                        this.groundedBooleanFluents.add((BoolPredicate) f);
                }
            } catch (Exception e) {
                System.err.println("[ENHSP] WARNING: getAllFluents() unavailable; "
                        + "falling back to PDDLProblem.booleanFluents (static).");
                this.groundedBooleanFluents = new HashSet<>(PDDLProblem.booleanFluents);
            }
            if (this.groundedBooleanFluents.isEmpty())
                System.err.println("[ENHSP] ERROR: No grounded boolean fluents found. "
                        + "Snowman heuristic will not work correctly.");

            if (stopAfterGrounding)
                System.exit(1);
            return Pair.of(localDomain, localProblem);
        } catch (Exception ex) {
            Logger.getLogger(ENHSP.class.getName()).log(Level.SEVERE, null, ex);
        }
        return null;
    }

    public void parsingDomainAndProblem(String[] args) {
        try {
            overallStart = System.currentTimeMillis();
            Pair<PDDLDomain, PDDLProblem> res = parseDomainProblem(
                    domainFile, problemFile, deltaExecution, System.out);
            domain = res.getKey();
            problem = res.getRight();
            if (pddlPlus) {
                res = parseDomainProblem(domainFile, problemFile, deltaHeuristic,
                        new PrintStream(new OutputStream() {
                            public void write(int b) {
                            }
                        }));
                heuristicProblem = res.getRight();
            } else {
                heuristicProblem = problem;
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void configurePlanner() {
        if (planner != null)
            setPlanner();
    }

    // ==================================================================================
    // Snowman preprocessing
    // ==================================================================================

    /**
     * Extracts maze topology and ball objects from the PDDL problem, then:
     * 1. Builds a dirMatrix (adjacency with direction labels) from the "next"
     * predicates.
     * 2. Computes all-pairs shortest path (Floyd-Warshall, used for character
     * reachability).
     * 3. Computes pushDistMatrix via a 0-1 BFS:
     * walk cost = 0, push cost = 1.
     * The multi-source variant (all valid character starting positions at cost 0)
     * gives the minimum pushes to move a ball from A to B regardless of where the
     * character starts — which is the correct relaxation for this heuristic.
     * 4. Identifies dead-end nodes (degree ≤ 1) for dead-end detection.
     */

    private int directionToInt(String dir) {
        // Removes any quotes or spaces and converts to lowercase
        String cleanDir = dir.replace("'", "").replace("\"", "").trim().toLowerCase();
        switch (cleanDir) {
            case "right":
            case "dir_right":
                return 0;
            case "left":
            case "dir_left":
                return 1;
            case "up":
            case "dir_up":
                return 2;
            case "down":
            case "dir_down":
                return 3;
            default:
                return -1;
        }
    }

    public void preprocessSnowman() {

        // --- Extract ball and location objects from PDDL ---
        PDDLObjects objects = problem.getProblemObjects();
        for (Object obj : objects.toArray()) {
            PDDLObject pddlObj = (PDDLObject) obj;
            String name = pddlObj.getName();
            String type = pddlObj.getType().getName();
            if (type.equals("ball")) {
                ballNames.add(name);
            } else if (type.equals("location")) {
                locationIndexMap.put(name, locationNames.size());
                locationNames.add(name);
            }
        }

        int N = locationNames.size();

        // --- Initialise distance matrix and direction matrix ---
        distanceMatrix = new int[N][N];
        dirMatrix = new int[N][N];
        for (int i = 0; i < N; i++) {
            java.util.Arrays.fill(distanceMatrix[i], models.SnowmanConfigurator.UNREACHABLE);
            java.util.Arrays.fill(dirMatrix[i], -1);
            distanceMatrix[i][i] = 0;
        }

        // --- Parse "next" predicates from the PDDL problem file ---
        try {
            String content = java.nio.file.Files.readString(java.nio.file.Path.of(this.problemFile));
            content = content.replaceAll("\\(", " ( ").replaceAll("\\)", " ) ").replaceAll("=", " = ");
            String[] tokens = content.split("\\s+");
            for (int i = 0; i < tokens.length; i++) {
                if (tokens[i].equalsIgnoreCase("next") && i + 3 < tokens.length) {
                    int u = getLocIdIgnoreCase(tokens[i + 1]);
                    int v = getLocIdIgnoreCase(tokens[i + 2]);
                    String dir = tokens[i + 3];
                    if (u != -1 && v != -1) {
                        distanceMatrix[u][v] = 1;
                        dirMatrix[u][v] = directionToInt(dir);
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("[SnowmanConfig] WARNING: token parser failed for 'next' predicates.");
        }

        // --- Identify dead-end and corner nodes ---
        isDeadEndNode = new boolean[N];
        isCornerNode = new boolean[N];
        int deadEndCount = 0;
        int cornerCount = 0;
        for (int i = 0; i < N; i++) {
            int degree = 0;
            for (int j = 0; j < N; j++)
                if (i != j && distanceMatrix[i][j] == 1)
                    degree++;
            isDeadEndNode[i] = (degree <= 1);
            if (isDeadEndNode[i])
                deadEndCount++;
        }
        // Corner/tunnel detection requires adjacency lists — done after adjList build

        // --- Build adjacency lists from dirMatrix ---
        adjList = new int[N][];
        adjDir = new int[N][];
        oppAdj = new int[N][];
        for (int u = 0; u < N; u++) {
            int deg = 0;
            for (int v = 0; v < N; v++)
                if (dirMatrix[u][v] != -1)
                    deg++;
            adjList[u] = new int[deg];
            adjDir[u] = new int[deg];
            oppAdj[u] = new int[deg];
            int idx = 0;
            for (int v = 0; v < N; v++) {
                if (dirMatrix[u][v] != -1) {
                    adjList[u][idx] = v;
                    adjDir[u][idx] = dirMatrix[u][v];
                    idx++;
                }
            }
            // Fill oppAdj: for edge u→v with dir d, find the neighbor w of u with dir
            // opposite(d)
            for (int i = 0; i < deg; i++) {
                int d = adjDir[u][i];
                int oppD = (d % 2 == 0) ? d + 1 : d - 1;
                oppAdj[u][i] = -1;
                for (int j = 0; j < deg; j++) {
                    if (adjDir[u][j] == oppD) {
                        oppAdj[u][i] = adjList[u][j];
                        break;
                    }
                }
            }
        }
        System.out.println("[ENHSP] Adjacency lists built (max degree: "
                + java.util.Arrays.stream(adjList).mapToInt(a -> a.length).max().orElse(0) + ").");

        // --- Corner and tunnel detection using adjList/adjDir ---
        for (int u = 0; u < N; u++) {
            if (adjList[u].length == 2) {
                int d0 = adjDir[u][0];
                int d1 = adjDir[u][1];
                // Orthogonal: one horizontal (0=right,1=left) + one vertical (2=up,3=down)
                boolean h0 = (d0 <= 1), h1 = (d1 <= 1);
                if (h0 != h1) {
                    isCornerNode[u] = true;
                    cornerCount++;
                }
                // Collinear (same axis) = tunnel — no special tracking needed
            }
        }
        System.out.println("[ENHSP] Topology: " + deadEndCount + " dead-ends, "
                + cornerCount + " corners.");

        // --- All-Pairs Shortest Path (Floyd-Warshall) for character reachability ---
        for (int k = 0; k < N; k++) {
            for (int i = 0; i < N; i++) {
                if (distanceMatrix[i][k] >= SnowmanConfigurator.UNREACHABLE)
                    continue;
                for (int j = 0; j < N; j++) {
                    if (distanceMatrix[k][j] >= SnowmanConfigurator.UNREACHABLE)
                        continue;
                    int d = distanceMatrix[i][k] + distanceMatrix[k][j];
                    if (d < distanceMatrix[i][j])
                        distanceMatrix[i][j] = d;
                }
            }
        }

        // =====================================================================
        // 0-1 BFS for push-distance matrix
        //
        // 0-1 BFS with cost 0 for walks and cost 1 for pushes.
        // → pushDistMatrix[a][b] = min pushes only (character walking is free).
        // → always a lower bound on h*(n) → admissible.
        //
        // Multi-source variant: all (char, ball=startBall) states with char ≠ ball
        // are initialised to cost 0, capturing the relaxation that the character
        // can start anywhere without extra cost.
        // =====================================================================

        // walkDistMatrix removed — identical to distanceMatrix
        // (Floyd-Warshall)

        pushDistMatrix = new int[N][N];
        for (int i = 0; i < N; i++)
            Arrays.fill(pushDistMatrix[i], SnowmanConfigurator.UNREACHABLE);

        int[] bfsDist01 = new int[N * N];

        // ArrayDeque is used here because this is a one-time preprocessing step
        // (not on the hot path); correctness and clarity take priority.
        ArrayDeque<Integer> deque = new ArrayDeque<>(N * N);

        boolean[] visited01 = new boolean[N * N];

        for (int startBall = 0; startBall < N; startBall++) {
            pushDistMatrix[startBall][startBall] = 0;

            // Reset delle strutture per la nuova palla di partenza
            Arrays.fill(bfsDist01, models.SnowmanConfigurator.UNREACHABLE);

            // Reset visitati a false per questa ricerca
            Arrays.fill(visited01, false);

            deque.clear();

            // Inseriamo in coda tutte le posizioni di partenza del personaggio a costo 0
            for (int startChar = 0; startChar < N; startChar++) {
                if (startChar == startBall)
                    continue;
                int s = startChar * N + startBall;

                bfsDist01[s] = 0;
                deque.addFirst(s); // Walk = costo 0, va in testa
            }

            // Inizio esplorazione
            while (!deque.isEmpty()) {
                int state = deque.pollFirst();

                // Se questo stato è già stato espanso, saltiamo l'iterazione
                if (visited01[state]) {
                    continue;
                }
                visited01[state] = true;

                int c = state / N; // Posizione attuale del personaggio
                int b = state % N; // Posizione attuale della palla
                int d = bfsDist01[state]; // Costo (numero di push) per arrivare qui

                for (int ai = 0; ai < adjList[c].length; ai++) {
                    int nextC = adjList[c][ai];

                    if (nextC == b) {
                        // PUSH: find nextB via adjList
                        int pushDir = dirMatrix[c][b];
                        int targetNextB = -1;
                        for (int aii = 0; aii < adjList[b].length; aii++) {
                            if (adjDir[b][aii] == pushDir) {
                                targetNextB = adjList[b][aii];
                                break;
                            }
                        }
                        if (targetNextB != -1) {
                            int nextState = b * N + targetNextB;
                            int newDist = d + 1;
                            if (newDist < bfsDist01[nextState]) {
                                bfsDist01[nextState] = newDist;
                                deque.addLast(nextState);
                                if (newDist < pushDistMatrix[startBall][targetNextB])
                                    pushDistMatrix[startBall][targetNextB] = newDist;
                            }
                        }
                    } else {
                        int nextState = nextC * N + b;
                        if (d < bfsDist01[nextState]) {
                            bfsDist01[nextState] = d;
                            deque.addFirst(nextState);
                        }
                    }
                }
            }
        }

        System.out.println("[ENHSP] Push-distance matrix computed (0-1 BFS, push=1, walk=0).");

        targetSnowmen = ballNames.size() / 3;
        if (targetSnowmen == 0 && !ballNames.isEmpty()) {
            System.err.println("[ENHSP] WARNING: " + ballNames.size()
                    + " balls found — insufficient for 1 snowman. Forcing targetSnowmen=1.");
            targetSnowmen = 1;
        }

        System.out.println("=== Snowman Topological Preprocessing ===");
        System.out.println("Balls:     " + ballNames.size() + " " + ballNames);
        System.out.println("Locations: " + N);
        System.out.println("Dead-ends: " + deadEndCount + "/" + N);
        System.out.println("Targets:   " + targetSnowmen + " snowman/men");
        System.out.println("=========================================");
    }

    // ==================================================================================
    // Planning loop (unchanged from original ENHSP)
    // ==================================================================================

    public void planning() {
        try {
            printStats();
            setHeuristic();
            do {
                LinkedList<?> sp = search();
                if (sp == null)
                    return;
                depthLimit = endGValue;
                if (anyTime)
                    System.out.println("NEW COST ===> " + depthLimit);
                sp = null;
                System.gc();
            } while (anyTime);
        } catch (Exception ex) {
            Logger.getLogger(ENHSP.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    // ==================================================================================
    // Command-line argument parsing (unchanged)
    // ==================================================================================

    public void parseInput(String[] args) {
        Options options = new Options();
        options.addRequiredOption("o", "domain", true, "PDDL domain file");
        options.addRequiredOption("f", "problem", true, "PDDL problem file");
        options.addOption("planner", true, "Fast Preconfigured Planner.");
        options.addOption("h", true,
                "heuristic (default hadd): aibr | hadd | hradd | hmax | hrmax | hmrp | "
                        + "blcost | blind | snowman");
        options.addOption("s", true,
                "search strategy (default gbfs): gbfs | WAStar | wa_star_4 | ida | ucs");
        options.addOption("ties", true, "tie-breaking: larger_g | smaller_g | arbitrary");
        options.addOption("dp", "delta_planning", true, "planning delta");
        options.addOption("de", "delta_execution", true, "execution delta");
        options.addOption("dh", "delta_heuristic", true, "heuristic delta");
        options.addOption("dv", "delta_validation", true, "validation delta");
        options.addOption("d", "delta", true, "override all deltas");
        options.addOption("epsilon", true, "epsilon separation");
        options.addOption("wg", true, "g-value weight");
        options.addOption("wh", true, "h-value weight");
        options.addOption("sjr", false, "save search space as JSON");
        options.addOption("ha", "helpful-actions", true, "helpful actions pruning");
        options.addOption("pe", "print-events-plan", false, "print events");
        options.addOption("ht", "helpful-transitions", true, "helpful transitions");
        options.addOption("sp", true, "save plan to file");
        options.addOption("pt", false, "print state trajectory");
        options.addOption("dap", false, "disable AIBR preprocessing");
        options.addOption("red", "redundant_constraints", true, "no | brute | smart");
        options.addOption("gro", "grounding", true, "internal | fd | metricff | naive");
        options.addOption("dl", true, "plan-cost bound");
        options.addOption("k", true, "max subdomains");
        options.addOption("anytime", false, "anytime mode");
        options.addOption("timeout", true, "timeout (seconds)");
        options.addOption("stopgro", false, "stop after grounding");
        options.addOption("ival", false, "internal validation");
        options.addOption("sdac", false, "state-dependent action cost");
        options.addOption("onlyplan", false, "print plan only");

        CommandLineParser parser = new DefaultParser();
        try {
            CommandLine cmd = parser.parse(options, args);
            domainFile = cmd.getOptionValue("o");
            problemFile = cmd.getOptionValue("f");
            planner = cmd.getOptionValue("planner");
            heuristic = cmd.getOptionValue("h");
            if (heuristic == null)
                heuristic = "hadd";
            searchEngineString = cmd.getOptionValue("s");
            if (searchEngineString == null)
                searchEngineString = "gbfs";
            tieBreaking = cmd.getOptionValue("ties");
            deltaPlanning = cmd.getOptionValue("dp");
            if (deltaPlanning == null)
                deltaPlanning = "1.0";
            String ro = cmd.getOptionValue("red");
            redundantConstraints = (ro != null) ? ro : "no";
            String go = cmd.getOptionValue("gro");
            groundingType = (go != null) ? go : "internal";
            deltaExecution = cmd.getOptionValue("de");
            if (deltaExecution == null)
                deltaExecution = "1.0";
            deltaHeuristic = cmd.getOptionValue("dh");
            if (deltaHeuristic == null)
                deltaHeuristic = "1.0";
            deltaValidation = cmd.getOptionValue("dv");
            if (deltaValidation == null)
                deltaValidation = "1";
            String tmp = cmd.getOptionValue("dl");
            depthLimit = (tmp != null) ? Float.parseFloat(tmp) : Float.NaN;
            String tos = cmd.getOptionValue("timeout");
            timeOut = (tos != null) ? Long.parseLong(tos) * 1000L : Long.MAX_VALUE;
            String delta = cmd.getOptionValue("delta");
            if (delta != null) {
                deltaHeuristic = deltaValidation = deltaPlanning = deltaExecution = delta;
            }
            hw = cmd.getOptionValue("wh");
            saving_json = cmd.hasOption("sjr");
            sdac = cmd.hasOption("sdac");
            helpfulActionsPruning = cmd.getOptionValue("ha") != null && "true".equals(cmd.getOptionValue("ha"));
            printEvents = cmd.hasOption("pe");
            savePlan = cmd.getOptionValue("sp");
            onlyPlan = cmd.hasOption("onlyplan");
            anyTime = cmd.hasOption("anytime");
            aibrPreprocessing = !cmd.hasOption("dap");
            stopAfterGrounding = cmd.hasOption("stopgro");
            helpfulTransitions = cmd.getOptionValue("ht") != null && "true".equals(cmd.getOptionValue("ht"));
            ignoreMetric = cmd.hasOption("im");
        } catch (ParseException exp) {
            System.err.println("Parsing failed: " + exp.getMessage());
            new HelpFormatter().printHelp("enhsp", options);
            System.exit(-1);
        }
    }

    public PDDLProblem getProblem() {
        return problem;
    }

    public void printStats() {
        System.out.println("|A|:" + getProblem().getActions().size());
        System.out.println("|P|:" + getProblem().getProcessesSet().size());
        System.out.println("|E|:" + getProblem().getEventsSet().size());
        if (pddlPlus) {
            System.out.println("Delta heuristic: " + deltaHeuristic);
            System.out.println("Delta planning:  " + deltaPlanning);
            System.out.println("Delta execution: " + deltaExecution);
            System.out.println("Delta validation:" + deltaValidation);
        }
    }

    private void setPlanner() {
        helpfulTransitions = false;
        helpfulActionsPruning = false;
        tieBreaking = "arbitrary";
        switch (planner) {
            case "sat-hmrp":
                heuristic = "hmrp";
                searchEngineString = "gbfs";
                break;
            case "sat-hmrph":
                heuristic = "hmrp";
                helpfulActionsPruning = true;
                searchEngineString = "gbfs";
                break;
            case "sat-hmrphj":
                heuristic = "hmrp";
                helpfulActionsPruning = true;
                helpfulTransitions = true;
                searchEngineString = "gbfs";
                break;
            case "sat-hadd":
                heuristic = "hadd";
                searchEngineString = "gbfs";
                tieBreaking = "smaller_g";
                break;
            case "sat-aibr":
                heuristic = "aibr";
                searchEngineString = "WAStar";
                break;
            case "sat-hradd":
                heuristic = "hradd";
                searchEngineString = "gbfs";
                tieBreaking = "smaller_g";
                break;
            case "opt-hmax":
                heuristic = "hmax";
                searchEngineString = "WAStar";
                tieBreaking = "larger_g";
                break;
            case "opt-hrmax":
                heuristic = "hrmax";
                searchEngineString = "WAStar";
                tieBreaking = "larger_g";
                break;
            case "opt-blind":
                heuristic = "blind";
                searchEngineString = "WAStar";
                tieBreaking = "larger_g";
                aibrPreprocessing = false;
                break;
            case "sat-blind":
                heuristic = "blind";
                searchEngineString = "gbfs";
                tieBreaking = "larger_g";
                aibrPreprocessing = false;
                break;
            case "snowman":
                heuristic = "snowman";
                searchEngineString = "WAStar";
                tieBreaking = "larger_g";
                helpfulActionsPruning = false;
                helpfulTransitions = false;
                aibrPreprocessing = false;
                break;
            default:
                System.out.println("! Unknown planner configuration; defaulting to gbfs+hadd.");
                heuristic = "hadd";
                searchEngineString = "gbfs";
                tieBreaking = "smaller_g";
        }
    }

    private void setHeuristic() {
        if (heuristic.equals("snowman")) {
            preprocessSnowman();
            h = new SnowmanHeuristic();
        } else {
            h = PDDLHeuristic.getHeuristic(heuristic, heuristicProblem,
                    redundantConstraints, helpfulActionsPruning, helpfulTransitions);
        }
    }

    // ==================================================================================
    // Search (unchanged from original ENHSP)
    // ==================================================================================

    private LinkedList<Pair<BigDecimal, Object>> search() throws Exception {
        LinkedList<Pair<BigDecimal, Object>> rawPlan = null;
        final PDDLSearchEngine searchEngine = new PDDLSearchEngine(problem, h);
        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                if (saving_json)
                    searchEngine.searchSpaceHandle.print_json(getProblem().getPddlFileReference() + ".sp_log");
            }
        });
        if (pddlPlus) {
            searchEngine.executionDelta = new BigDecimal(deltaExecution);
            searchEngine.processes = true;
            searchEngine.planningDelta = new BigDecimal(deltaPlanning);
        }
        searchEngine.saveSearchTreeAsJson = saving_json;
        if (tieBreaking != null) {
            switch (tieBreaking) {
                case "smaller_g":
                    searchEngine.tbRule = SearchEngine.TieBreaking.LOWERG;
                    break;
                case "larger_g":
                    searchEngine.tbRule = SearchEngine.TieBreaking.HIGHERG;
                    break;
                default:
                    System.out.println("Unknown tie-breaking rule; using ARBITRARY.");
                    // fall through
            }
        } else {
            tieBreaking = "arbitrary";
            searchEngine.tbRule = SearchEngine.TieBreaking.ARBITRARY;
        }
        if (hw != null)
            searchEngine.setWH(Float.parseFloat(hw));
        else
            searchEngine.setWH(1);
        if (!Float.isNaN(depthLimit))
            searchEngine.depthLimit = depthLimit;
        else
            searchEngine.depthLimit = Float.POSITIVE_INFINITY;
        searchEngine.helpfulActionsPruning = helpfulActionsPruning;

        if ("WAStar".equals(searchEngineString))
            rawPlan = searchEngine.WAStar(getProblem(), timeOut);
        else if ("wa_star_4".equals(searchEngineString)) {
            searchEngine.setWH(4);
            rawPlan = searchEngine.WAStar();
        } else if ("gbfs".equals(searchEngineString))
            rawPlan = searchEngine.gbfs(getProblem(), timeOut);
        else if ("gbfs_ha".equals(searchEngineString))
            rawPlan = searchEngine.gbfs(getProblem(), timeOut);
        else if ("ida".equals(searchEngineString))
            rawPlan = searchEngine.idastar(getProblem(), true);
        else if ("ucs".equals(searchEngineString))
            rawPlan = searchEngine.UCS(getProblem());
        else
            throw new RuntimeException("Unknown search strategy: " + searchEngineString);

        endGValue = searchEngine.currentG;
        overallPlanningTime = System.currentTimeMillis() - overallStart;
        printInfo(rawPlan, searchEngine);
        return rawPlan;
    }

    private void printInfo(LinkedList<Pair<BigDecimal, Object>> sp, PDDLSearchEngine se)
            throws CloneNotSupportedException {
        PDDLState s = (PDDLState) se.getLastState();
        if (sp != null) {
            System.out.println("Problem Solved\n");
            System.out.println("Found Plan:");
            printPlan(sp, pddlPlus, s, savePlan);
            System.out.println("\nPlan-Length:" + sp.size());
            planLength = sp.size();
        } else {
            System.out.println("Problem unsolvable");
        }
        if (pddlPlus && sp != null)
            System.out.println("Elapsed Time: " + s.time);
        System.out.println("Metric (Search):" + se.currentG);
        System.out.println("Planning Time (msec): " + overallPlanningTime);
        System.out.println("Heuristic Time (msec): " + se.getHeuristicCpuTime());
        System.out.println("Search Time (msec): " + se.getOverallSearchTime());
        System.out.println("Expanded Nodes:" + se.getNodesExpanded());
        System.out.println("States Evaluated:" + se.getNumberOfEvaluatedStates());
        System.out.println("Dead-Ends detected:" + se.deadEndsDetected);
        System.out.println("Duplicates detected:" + se.duplicatesNumber);
        if (saving_json)
            se.searchSpaceHandle.print_json(getProblem().getPddlFileReference() + ".sp_log");
    }

    private void printPlan(LinkedList<Pair<BigDecimal, Object>> plan, boolean temporal,
            PDDLState par, String fileName) {
        float i = 0f;
        Pair<BigDecimal, Object> previous = null;
        List<String> fileContent = new ArrayList<>();
        boolean startProcess = false;
        int size = plan.size(), j = 0;
        for (Pair<BigDecimal, Object> ele : plan) {
            j++;
            if (!temporal) {
                System.out.print(i + ": " + ele.getRight() + "\n");
                if (fileName != null)
                    fileContent.add(((TransitionGround) ele.getRight()).toString());
                i++;
            } else {
                TransitionGround t = (TransitionGround) ele.getRight();
                if (t.getSemantics() == TransitionGround.Semantics.PROCESS) {
                    if (!startProcess) {
                        previous = ele;
                        startProcess = true;
                    }
                    if (j == size && !onlyPlan)
                        System.out.println(previous.getLeft() + ": -----waiting---- [" + par.time + "]");
                } else {
                    if (t.getSemantics() != TransitionGround.Semantics.EVENT || printEvents) {
                        if (startProcess) {
                            startProcess = false;
                            if (!onlyPlan)
                                System.out.println(previous.getLeft() + ": -----waiting---- [" + ele.getLeft() + "]");
                        }
                        System.out.print(ele.getLeft() + ": " + ele.getRight() + "\n");
                        if (fileName != null)
                            fileContent.add(ele.getLeft() + ": " + t.toString());
                    } else {
                        if (j == size && !onlyPlan)
                            System.out.println(previous.getLeft() + ": -----waiting---- [" + ele.getLeft() + "]");
                    }
                }
            }
        }
        if (fileName != null) {
            try {
                Files.write(Path.of(fileName), fileContent);
            } catch (IOException ex) {
                Logger.getLogger(ENHSP.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    // ==================================================================================
    // SnowmanHeuristic inner class
    // ==================================================================================

    public class SnowmanHeuristic implements SearchHeuristic {

        // --- Tracker inner classes ---

        private class BallTracker {
            final Ball ballInstance;
            final List<BoolPredLocPair> locationPredicates;
            final NumFluent sizeFluent;
            final int fixedLocId; // fallback if grounder misses this ball
            final int fixedSize;

            BallTracker(Ball ball, List<BoolPredLocPair> lps, NumFluent sf, int fixedLoc, int fixedSz) {
                this.ballInstance = ball;
                this.locationPredicates = lps;
                this.sizeFluent = sf;
                this.fixedLocId = fixedLoc;
                this.fixedSize = fixedSz;
            }
        }

        private class BoolPredLocPair {
            final BoolPredicate predicate;
            final int locId;

            BoolPredLocPair(BoolPredicate p, int l) {
                predicate = p;
                locId = l;
            }
        }

        private class CellTracker {
            // snowPredicate may be null if the grounder did not produce this fluent.
            // In that case the cell is not tracked (treated as snow-free → admissible
            // under-estimate).
            BoolPredicate snowPredicate;
            final int locId;

            CellTracker(int l) {
                locId = l;
            }
        }

        private class CharacterTracker {
            final BoolPredicate predicate;
            final int locId;

            CharacterTracker(BoolPredicate p, int l) {
                predicate = p;
                locId = l;
            }
        }

        // Track "snowman_at" predicates from Domain_two.pddl
        private class SnowmanAtTracker {
            final BoolPredicate predicate;
            final int locId;

            SnowmanAtTracker(BoolPredicate p, int l) {
                predicate = p;
                locId = l;
            }
        }

        private final List<BallTracker> ballTrackers;
        private final List<CellTracker> cellTrackers;
        private final List<CharacterTracker> characterTrackers;
        private final List<SnowmanAtTracker> snowmanAtTrackers;
        private final List<Ball> preInstantiatedBalls;
        private final SnowmanConfigurator configurator;
        private final boolean allSnowTracked;

        // --- Zero-allocation working sets ---
        private final List<Ball> activeBallsList;
        private final List<Ball> remainingBallsList;
        private final int[] size1Count;
        private final int[] size2Count;
        private final int[] size3Count;
        private final boolean[] currentSnow;
        private final boolean[] initialSnowArray;
        private final int[] sizesBuffer;
        private final int[] maxPossibleSizeBuffer; // pre-allocated
        private final int fixedCharLoc;

        // --- Ball-state caching (skip configurator when state is unchanged) ---
        private long cachedBallFingerprint = 0L;
        private long cachedSnowFingerprint = 0L;
        private double cachedHBalls = 0.0;

        private int cachedRemainingTargets = 0;

        // --- Helpful transitions ---
        private Collection<TransitionGround> cachedHelpfulTransitions;

        // --- Per-state arrays ---
        int[] currentSizes;
        int[] currentLocs;

        // --- Zero-allocation flood-fill BFS for character reachability ---
        private final int[] charDist;
        private final boolean[] occupiedByBall;
        private final int[] floodFillQueue;

        // --- Warning flags (print at most once per instance) ---
        private boolean warnedInconsistentSnowmen = false;

        private int lastNumSnowmen = 0;
        private final boolean[] snowmanLocs = new boolean[locationNames.size()];

        // --- Campi per Gestione Plateau e Focus ---
        // Replaced globalMinHBalls with lastHBalls to prevent unbounded bias
        private double lastHBalls = -1.0;
        private int plateauCount = 0;
        private models.Grouping currentFocusGroup = null;
        private int activePhaseBallId = -1;
        private int lastFocusTarget = -1;

        // ==========================================================================
        // Constructor
        // ==========================================================================

        @SuppressWarnings("unchecked")
        public SnowmanHeuristic() {
            ballTrackers = new ArrayList<>();
            cellTrackers = new ArrayList<>();
            characterTrackers = new ArrayList<>();
            snowmanAtTrackers = new ArrayList<>();
            preInstantiatedBalls = new ArrayList<>();

            int N = locationNames.size();
            currentSnow = new boolean[N];

            // --- Index NumFluents for ball_size ---
            Set<String> ballNamesSet = new HashSet<>(ballNames);
            Map<String, NumFluent> sizeFluentMap = new HashMap<>();
            for (Object nfObj : problem.getNumFluents()) {
                if (!(nfObj instanceof NumFluent))
                    continue;
                NumFluent nf = (NumFluent) nfObj;
                try {
                    List<?> terms = nf.getTerms();
                    if (terms == null || terms.isEmpty())
                        continue;
                    Object term0 = terms.get(0);
                    String bName = (term0 instanceof PDDLObject)
                            ? ((PDDLObject) term0).getName()
                            : term0.toString().trim();
                    if (bName != null && ballNamesSet.contains(bName))
                        sizeFluentMap.put(bName, nf);
                } catch (Exception ignored) {
                }
            }

            // --- Index BoolPredicates for ball_at, character_at, snow ---
            Map<String, List<BoolPredLocPair>> ballAtMap = new HashMap<>();
            for (String bn : ballNames)
                ballAtMap.put(bn, new ArrayList<>());
            Map<String, CellTracker> cellTrackerMap = new HashMap<>();

            for (BoolPredicate bf : ENHSP.this.groundedBooleanFluents) {
                String pName = bf.getName();
                List<PDDLObject> terms = bf.getTerms();
                if (pName.equals("ball_at") && terms.size() >= 2) {
                    String bName = terms.get(0).getName();
                    String lName = terms.get(1).getName();
                    if (ballNames.contains(bName)) {
                        int locId = getLocationIndex(lName);
                        if (locId != -1)
                            ballAtMap.get(bName).add(new BoolPredLocPair(bf, locId));
                    }
                } else if (pName.equals("character_at") && terms.size() >= 1) {
                    int locId = getLocationIndex(terms.get(0).getName());
                    if (locId != -1)
                        characterTrackers.add(new CharacterTracker(bf, locId));
                } else if (pName.equals("snow") && terms.size() >= 1) {
                    String lName = terms.get(0).getName();
                    int locId = getLocationIndex(lName);
                    if (locId != -1) {
                        CellTracker ct = cellTrackerMap.computeIfAbsent(lName, k -> new CellTracker(locId));
                        ct.snowPredicate = bf;
                    }
                } else if (pName.equals("snowman_at") && terms.size() >= 1) {
                    // Track snowman_at predicates
                    int locId = getLocationIndex(terms.get(0).getName());
                    if (locId != -1)
                        snowmanAtTrackers.add(new SnowmanAtTracker(bf, locId));
                }
            }
            cellTrackers.addAll(cellTrackerMap.values());

            // --- Parse initial state from PDDL file (token-based, fallback) ---
            Map<String, Integer> initialBallPositions = new HashMap<>();
            Map<String, Integer> initialBallSizes = new HashMap<>();
            int initialCharPos = -1;
            Set<Integer> initialSnowPositions = new HashSet<>();
            try {
                String content = java.nio.file.Files.readString(java.nio.file.Path.of(ENHSP.this.problemFile));
                content = content.replaceAll("\\(", " ( ").replaceAll("\\)", " ) ").replaceAll("=", " = ");
                String[] tokens = content.split("\\s+");
                for (int i = 0; i < tokens.length; i++) {
                    if (tokens[i].equalsIgnoreCase("snow") && i + 1 < tokens.length) {
                        int lid = getLocIdIgnoreCase(tokens[i + 1]);
                        if (lid != -1)
                            initialSnowPositions.add(lid);
                    } else if (tokens[i].equalsIgnoreCase("ball_size") && i + 1 < tokens.length) {
                        String bn = getBallNameIgnoreCase(tokens[i + 1]);
                        if (bn != null) {
                            for (int j = i + 2; j <= i + 4 && j < tokens.length; j++) {
                                try {
                                    initialBallSizes.put(bn, Integer.parseInt(tokens[j]));
                                    break;
                                } catch (NumberFormatException ignored) {
                                }
                            }
                        }
                    } else if (tokens[i].equalsIgnoreCase("ball_at") && i + 2 < tokens.length) {
                        String bn = getBallNameIgnoreCase(tokens[i + 1]);
                        int lid = getLocIdIgnoreCase(tokens[i + 2]);
                        if (bn != null && lid != -1)
                            initialBallPositions.put(bn, lid);
                    } else if (tokens[i].equalsIgnoreCase("character_at") && i + 1 < tokens.length) {
                        initialCharPos = getLocIdIgnoreCase(tokens[i + 1]);
                    }
                }
            } catch (Exception e) {
                System.err.println("[SnowmanHeuristic] WARNING: fallback token parser failed.");
            }

            // Override with PPMAJAL API if available.
            Iterable<?> initPreds = problem.getPredicatesInvolvedInInit();
            if (initPreds != null) {
                for (Object obj : initPreds) {
                    if (!(obj instanceof BoolPredicate))
                        continue;
                    BoolPredicate bp = (BoolPredicate) obj;
                    String pName = bp.getName();
                    List<PDDLObject> terms = bp.getTerms();
                    if (pName.equals("ball_at") && terms.size() >= 2) {
                        int lid = getLocationIndex(terms.get(1).getName());
                        if (lid != -1)
                            initialBallPositions.put(terms.get(0).getName(), lid);
                    } else if (pName.equals("character_at") && terms.size() >= 1) {
                        initialCharPos = getLocationIndex(terms.get(0).getName());
                    } else if (pName.equals("snow") && terms.size() >= 1) {
                        int lid = getLocationIndex(terms.get(0).getName());
                        if (lid != -1)
                            initialSnowPositions.add(lid);
                    }
                }
            }

            this.fixedCharLoc = initialCharPos;
            initialSnowArray = new boolean[N];
            for (int lid : initialSnowPositions)
                if (lid >= 0 && lid < N)
                    initialSnowArray[lid] = true;

            long trackedSnow = cellTrackers.stream().filter(ct -> ct.snowPredicate != null).count();
            this.allSnowTracked = (trackedSnow >= initialSnowPositions.size());
            System.out.println("[SnowmanHeuristic] Snow cells in PDDL: " + initialSnowPositions.size()
                    + ", tracked: " + trackedSnow
                    + (allSnowTracked ? " [FULL]" : " [PARTIAL - snow budget check disabled]"));

            // --- Build BallTrackers ---
            int intId = 0;
            for (String bName : ballNames) {
                Ball ball = new Ball(bName, intId, 1, -1);
                preInstantiatedBalls.add(ball);
                int defLoc = initialBallPositions.getOrDefault(bName, -1);
                int defSize = initialBallSizes.getOrDefault(bName, 1);
                ballTrackers.add(new BallTracker(ball, ballAtMap.get(bName),
                        sizeFluentMap.get(bName), defLoc, defSize));
                intId++;
            }

            // --- Instantiate configurator ---
            configurator = new SnowmanConfigurator(N, preInstantiatedBalls,
                    targetSnowmen, distanceMatrix);
            configurator.setPushDistMatrix(pushDistMatrix);
            configurator.setAdjacencyLists(adjList, adjDir);
            // precomputeStaticSnowDetours removed — computeJointSnow
            // is now fully dynamic using activeSnowCells

            // --- Pre-allocate working arrays ---
            int numBalls = preInstantiatedBalls.size();
            activeBallsList = new ArrayList<>(numBalls);
            remainingBallsList = new ArrayList<>(numBalls);
            sizesBuffer = new int[numBalls];
            maxPossibleSizeBuffer = new int[numBalls];
            currentSizes = new int[numBalls];
            currentLocs = new int[numBalls];
            size1Count = new int[N];
            size2Count = new int[N];
            size3Count = new int[N];
            charDist = new int[N];
            occupiedByBall = new boolean[N];
            floodFillQueue = new int[N];

            System.out.println("[SnowmanHeuristic] Initialized: "
                    + ballTrackers.size() + " ball trackers, "
                    + cellTrackers.size() + " snow trackers, "
                    + characterTrackers.size() + " character trackers.");
        }

        // ==========================================================================
        // computeEstimate — main heuristic entry-point
        // ==========================================================================

        @Override
        public float computeEstimate(State state) {
            try {

                // ----- 1. Update snow -----
                Arrays.fill(currentSnow, false);
                for (CellTracker ct : cellTrackers) {
                    if (ct.snowPredicate != null && ct.snowPredicate.isSatisfied(state))
                        currentSnow[ct.locId] = true;
                }
                // Fallback: if no snow predicate fired, assume initial snow configuration.
                // This is admissible (initial snow ≥ remaining snow → lower bound).
                boolean anySnow = false;
                for (boolean s : currentSnow) {
                    if (s) {
                        anySnow = true;
                        break;
                    }
                }
                if (!anySnow)
                    System.arraycopy(initialSnowArray, 0, currentSnow, 0, currentSnow.length);
                configurator.updateEnvironment(currentSnow);

                // ----- 2. Update ball positions and sizes -----
                activeBallsList.clear();
                remainingBallsList.clear();
                Arrays.fill(size1Count, 0);
                Arrays.fill(size2Count, 0);
                Arrays.fill(size3Count, 0);

                for (BallTracker bt : ballTrackers) {
                    int s = bt.fixedSize;
                    if (bt.sizeFluent != null) {
                        Double val = bt.sizeFluent.eval(state);
                        if (val != null && val.intValue() > 0)
                            s = val.intValue();
                    }
                    int locId = -1;
                    for (BoolPredLocPair lp : bt.locationPredicates) {
                        if (lp.predicate.isSatisfied(state)) {
                            locId = lp.locId;
                            break;
                        }
                    }

                    // Se la palla è sparita (pupazzo costruito), la ignoriamo
                    if (locId < 0) {
                        continue;
                    }

                    int bid = bt.ballInstance.getIntId();
                    currentSizes[bid] = s;
                    currentLocs[bid] = locId;
                    activeBallsList.add(preInstantiatedBalls.get(bid));
                    if (s == 1)
                        size1Count[locId]++;
                    else if (s == 2)
                        size2Count[locId]++;
                    else if (s == 3)
                        size3Count[locId]++;
                }

                // Calcoliamo quanti pupazzi sono già stati confermati dall'azione build_snowman
                int missingBalls = ballTrackers.size() - activeBallsList.size();
                int builtFromMissing = missingBalls / 3;

                // ----- 3. Dead-end: not enough balls -----
                // Scaliamo il target in base ai pupazzi già confermati
                if (activeBallsList.size() < 3 * (targetSnowmen - builtFromMissing))
                    return Float.POSITIVE_INFINITY;

                // ----- 3.1 Identify completed snowmen and remaining balls -----
                int completedSnowmen = builtFromMissing; // Includiamo subito quelli completati e rimossi dalla mappa
                for (int loc = 0; loc < size1Count.length; loc++) {
                    int full = Math.min(size1Count[loc], Math.min(size2Count[loc], size3Count[loc]));
                    completedSnowmen += full;
                    size1Count[loc] -= full;
                    size2Count[loc] -= full;
                    size3Count[loc] -= full;
                }

                if (completedSnowmen > lastNumSnowmen) {
                    // Use snowman_at predicates when available
                    if (!snowmanAtTrackers.isEmpty()) {
                        for (SnowmanAtTracker sat : snowmanAtTrackers) {
                            if (sat.predicate.isSatisfied(state)) {
                                snowmanLocs[sat.locId] = true;
                            }
                        }
                    } else {
                        // Use the memory of the last valid target
                        if (lastFocusTarget != -1) {
                            snowmanLocs[lastFocusTarget] = true;
                        }
                    }
                    updatePushDistMatrixWithSnowmen(snowmanLocs);
                    lastNumSnowmen = completedSnowmen;
                    configurator.clearCache();

                    // RESET DOPO IL TRAGUARDO (Fase 4)
                    plateauCount = 0;
                    lastHBalls = -1.0;
                    currentFocusGroup = null;
                }
                for (Ball b : activeBallsList) {
                    int loc = currentLocs[b.getIntId()];
                    int s = currentSizes[b.getIntId()];
                    if (s == 1 && size1Count[loc] > 0) {
                        size1Count[loc]--;
                        remainingBallsList.add(b);
                    } else if (s == 2 && size2Count[loc] > 0) {
                        size2Count[loc]--;
                        remainingBallsList.add(b);
                    } else if (s == 3 && size3Count[loc] > 0) {
                        size3Count[loc]--;
                        remainingBallsList.add(b);
                    }
                }

                int remainingTargets = targetSnowmen - completedSnowmen;
                if (remainingTargets < 0) {
                    if (!warnedInconsistentSnowmen) {
                        System.err.println("[SnowmanHeuristic] WARNING: completedSnowmen ("
                                + completedSnowmen + ") > targetSnowmen. Inconsistent state.");
                        warnedInconsistentSnowmen = true;
                    }
                    remainingTargets = 0;
                }

                // ----- 3.1.2 Global snow budget dead-end check (EARLY PRUNING) -----
                if (remainingTargets > 0) {
                    int nBalls = remainingBallsList.size();
                    for (int i = 0; i < nBalls; i++)
                        sizesBuffer[i] = currentSizes[remainingBallsList.get(i).getIntId()];
                    Arrays.sort(sizesBuffer, 0, nBalls);
                    for (int lo = 0, hi = nBalls - 1; lo < hi; lo++, hi--) {
                        int tmp = sizesBuffer[lo];
                        sizesBuffer[lo] = sizesBuffer[hi];
                        sizesBuffer[hi] = tmp;
                    }
                    int totalMissingSnow = 0;
                    int rt = remainingTargets;
                    for (int i = 0; i < rt && i < nBalls; i++)
                        totalMissingSnow += Math.max(0, 3 - sizesBuffer[i]);
                    // Only Role 3 (base) needs up to 2 snow, Role 2 (middle) needs up to 1.
                    // If we have RT targets, we need RT bases and RT middles.
                    for (int i = rt; i < 2 * rt && i < nBalls; i++)
                        totalMissingSnow += Math.max(0, 2 - sizesBuffer[i]);

                    if (allSnowTracked && totalMissingSnow > configurator.getNumActiveSnowCells())
                        return Float.POSITIVE_INFINITY;

                    // ----- Spatial Snow Budgeting -----
                    if (remainingTargets > 0 && allSnowTracked) {
                        int rbSize = remainingBallsList.size();
                        for (int i = 0; i < rbSize; i++) {
                            Ball b = remainingBallsList.get(i);
                            int loc = currentLocs[b.getIntId()];
                            int sz = currentSizes[b.getIntId()];

                            boolean canReachSnow = false;
                            for (int s = 0; s < currentSnow.length; s++) {
                                if (currentSnow[s]
                                        && pushDistMatrix[loc][s] < models.SnowmanConfigurator.UNREACHABLE) {
                                    canReachSnow = true;
                                    break;
                                }
                            }
                            maxPossibleSizeBuffer[i] = canReachSnow ? 3 : sz;
                        }

                        Arrays.sort(maxPossibleSizeBuffer, 0, rbSize);

                        // Verify we can form enough bases (size 3)
                        boolean canFormBases = true;
                        for (int i = 0; i < remainingTargets && i < rbSize; i++) {
                            if (maxPossibleSizeBuffer[rbSize - 1 - i] < 3) {
                                canFormBases = false;
                                break;
                            }
                        }
                        if (!canFormBases)
                            return Float.POSITIVE_INFINITY;

                        // Verify we can form enough middles (size >= 2)
                        boolean canFormMiddles = true;
                        for (int i = remainingTargets; i < 2 * remainingTargets && i < rbSize; i++) {
                            if (maxPossibleSizeBuffer[rbSize - 1 - i] < 2) {
                                canFormMiddles = false;
                                break;
                            }
                        }
                        if (!canFormMiddles)
                            return Float.POSITIVE_INFINITY;
                    }
                }

                // ----- 3.1.5 Isolated-node and corner dead-end check -----
                if (remainingTargets > 0) {
                    int mobileBalls = remainingBallsList.size();
                    for (Ball b : remainingBallsList) {
                        int loc = currentLocs[b.getIntId()];
                        int s = currentSizes[b.getIntId()];

                        if (isDeadEndNode[loc]) {
                            // Dead-end (degree ≤ 1): ball is stuck unless it can stack
                            boolean trapped;
                            if (s == 1) {
                                trapped = true;
                            } else if (s == 2) {
                                boolean hasSize3 = false;
                                for (Ball other : remainingBallsList) {
                                    if (currentLocs[other.getIntId()] == loc
                                            && currentSizes[other.getIntId()] == 3) {
                                        hasSize3 = true;
                                        break;
                                    }
                                }
                                trapped = !hasSize3;
                            } else {
                                trapped = false; // size-3 at dead-end may still serve as base
                            }
                            if (trapped)
                                mobileBalls--;

                        } else if (isCornerNode[loc] && s < 3 && !snowmanLocs[loc]) {
                            // Corner (degree = 2 with orthogonal dirs): a ball smaller than
                            // size-3 cannot leave. Check if there's a size-3 at the same
                            // corner that could serve as a base (making this a valid stack).
                            boolean hasSize3AtCorner = false;
                            for (Ball other : remainingBallsList) {
                                if (currentLocs[other.getIntId()] == loc
                                        && currentSizes[other.getIntId()] == 3) {
                                    hasSize3AtCorner = true;
                                    break;
                                }
                            }
                            // If no size-3 present, this ball is trapped regardless of its
                            // own size. This check is order-independent: each ball at the
                            // corner is individually decremented.
                            if (!hasSize3AtCorner)
                                mobileBalls--;
                        }
                    }
                    if (mobileBalls < 3 * remainingTargets)
                        return Float.POSITIVE_INFINITY;
                }

                // ----- 3.2 Character position -----
                int charLocId = fixedCharLoc;
                for (CharacterTracker qt : characterTrackers) {
                    if (qt.predicate.isSatisfied(state)) {
                        charLocId = qt.locId;
                        break;
                    }
                }

                // ----- 3.3 Dynamic character flood-fill (BFS respecting ball obstacles) -----
                if (charLocId >= 0 && !remainingBallsList.isEmpty()) {
                    Arrays.fill(charDist, SnowmanConfigurator.UNREACHABLE);
                    Arrays.fill(occupiedByBall, false);
                    for (Ball b : activeBallsList)
                        occupiedByBall[currentLocs[b.getIntId()]] = true;

                    // It incorrectly treated occupiedByBall as permanent block,
                    // but pushing onto an occupied cell is valid stacking in Snowman.
                    // The static corner check (frozenH_Static && frozenV_Static with snowmanLocs)
                    // is preserved in isDeadEndNode and provides correct hard pruning.

                    // Proposal 1: charDist BFS using adjList instead of O(N) scan
                    int head = 0, tail = 0;
                    charDist[charLocId] = 0;
                    floodFillQueue[tail++] = charLocId;
                    while (head < tail) {
                        int curr = floodFillQueue[head++];
                        int dist = charDist[curr];
                        for (int ai = 0; ai < adjList[curr].length; ai++) {
                            int next = adjList[curr][ai];
                            if (!occupiedByBall[next]
                                    && charDist[next] == SnowmanConfigurator.UNREACHABLE) {
                                charDist[next] = dist + 1;
                                floodFillQueue[tail++] = next;
                            }
                        }
                    }

                    // Dead-end: character cannot reach any push position for any remaining ball.
                    // Proposal 1: use adjList for adjacency check
                    boolean canReachAnyPushPos = false;
                    outer: for (Ball b : remainingBallsList) {
                        int bLoc = currentLocs[b.getIntId()];
                        for (int ai = 0; ai < adjList[bLoc].length; ai++) {
                            int adj = adjList[bLoc][ai];
                            if (charDist[adj] < SnowmanConfigurator.UNREACHABLE) {
                                canReachAnyPushPos = true;
                                break outer;
                            }
                        }
                    }
                    if (!canReachAnyPushPos)
                        return Float.POSITIVE_INFINITY;
                }

                // ----- 4. Core heuristic computation -----
                double h_balls = 0.0;

                if (remainingTargets > 0) {

                    // 4.2 Ball-state caching
                    // Note: fingerprint is order-sensitive, but remainingBallsList iteration order
                    // is stable within a single computeEstimate call (same list built top-down).
                    long ballFP = 0L;
                    for (Ball b : remainingBallsList)
                        ballFP = ballFP * 131L + currentLocs[b.getIntId()] * 7L + currentSizes[b.getIntId()];
                    long snowFP = configurator.getSnowFingerprint();

                    if (ballFP == cachedBallFingerprint
                            && snowFP == cachedSnowFingerprint
                            && remainingTargets == cachedRemainingTargets
                            && cachedHBalls < SnowmanConfigurator.UNREACHABLE) {
                        h_balls = cachedHBalls;
                    } else {
                        SnowmanConfigurator.GroupingScore score = configurator.getBestScore(
                                remainingBallsList, remainingTargets, currentSizes, currentLocs);
                        h_balls = score.low;
                        cachedBallFingerprint = ballFP;
                        cachedSnowFingerprint = snowFP;
                        cachedHBalls = h_balls;
                        cachedRemainingTargets = remainingTargets;
                    }

                    // Controllo Dead-end PRIMA del plateau
                    if (h_balls >= SnowmanConfigurator.UNREACHABLE)
                        return Float.POSITIVE_INFINITY;

                    // Simple per-call comparison instead of monotonic globalMinHBalls
                    if (Math.abs(h_balls - lastHBalls) < 0.001) {
                        plateauCount++;
                    } else {
                        plateauCount = 0;
                        lastHBalls = h_balls;
                    }

                    // 4.3 Approach-cost dead-end check (static APSP, conservative).
                    // move_character has cost 0, so minApproach is NOT added to h_balls.
                    if (charLocId >= 0 && !remainingBallsList.isEmpty()) {
                        int minApproach = SnowmanConfigurator.UNREACHABLE;
                        for (Ball b : remainingBallsList) {
                            int d = distanceMatrix[charLocId][currentLocs[b.getIntId()]];
                            if (d < minApproach)
                                minApproach = d;
                        }
                        if (minApproach >= SnowmanConfigurator.UNREACHABLE)
                            return Float.POSITIVE_INFINITY;
                    }
                }

                if (helpfulTransitions && h_balls < Float.POSITIVE_INFINITY)
                    cachedHelpfulTransitions = computeHelpfulTransitions(state);
                else
                    cachedHelpfulTransitions = null;
                return (float) h_balls;

            } catch (Exception e) {
                System.err.println("[SnowmanHeuristic] EXCEPTION in computeEstimate: " + e.getMessage());
                e.printStackTrace();
                return Float.POSITIVE_INFINITY;
            }
        }

        private Collection<TransitionGround> computeHelpfulTransitions(State state) {
            Collection<TransitionGround> allTrans = problem.getTransitions();

            // Usiamo 100 come soglia perché ora il contatore è estremamente robusto
            if (currentFocusGroup == null || plateauCount > 100) {
                return allTrans;
            }

            int charLocId = fixedCharLoc;
            for (CharacterTracker qt : characterTrackers) {
                if (qt.predicate.isSatisfied(state)) {
                    charLocId = qt.locId;
                    break;
                }
            }

            List<TransitionGround> helpful = new ArrayList<>();

            for (TransitionGround t : allTrans) {
                String tName = t.getName();
                String actionString = t.toString();

                if (tName.startsWith("move_character") || tName.startsWith("build") || tName.startsWith("goal")) {
                    helpful.add(t);
                } else if (tName.startsWith("move_ball")) {
                    boolean isHelpful = false;

                    Ball activeBall = null;
                    if (activePhaseBallId != -1) {
                        for (Ball b : remainingBallsList) {
                            if (b.getIntId() == activePhaseBallId) {
                                activeBall = b;
                                break;
                            }
                        }
                    }

                    if (activeBall != null && actionString.contains(activeBall.getId())) {
                        isHelpful = true;
                    } else {
                        for (Ball b : remainingBallsList) {
                            if (actionString.contains(b.getId())) {
                                int bLoc = currentLocs[b.getIntId()];
                                if (charLocId >= 0 && distanceMatrix[charLocId][bLoc] <= 3) {
                                    isHelpful = true;
                                    break;
                                }
                                if (activeBall != null) {
                                    int activeLoc = currentLocs[activeBall.getIntId()];
                                    if (distanceMatrix[bLoc][activeLoc] <= 4) {
                                        isHelpful = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if (isHelpful) {
                        helpful.add(t);
                    }
                } else {
                    helpful.add(t);
                }
            }

            // Fallback: If the filter leaves us only with empty walking actions, use all
            // transitions
            boolean hasProgress = false;
            for (TransitionGround t : helpful) {
                if (!t.getName().startsWith("move_character")) {
                    hasProgress = true;
                    break;
                }
            }

            if (!hasProgress) {
                return allTrans;
            }

            return helpful;
        }

        @Override
        public Collection<TransitionGround> getAllTransitions() {
            return problem.getTransitions();
        }

        @Override
        public Object[] getTransitions(boolean arg0) {
            Collection<TransitionGround> x = (helpfulTransitions && cachedHelpfulTransitions != null)
                    ? cachedHelpfulTransitions
                    : problem.getTransitions();
            return x.toArray(new TransitionGround[0]);
        }

        public void updatePushDistMatrixWithSnowmen(boolean[] snowmanLocs) {
            int N = locationNames.size();

            // Reset della matrice
            for (int i = 0; i < N; i++)
                Arrays.fill(pushDistMatrix[i], models.SnowmanConfigurator.UNREACHABLE);

            int[] bfsDist01 = new int[N * N];
            ArrayDeque<Integer> deque = new ArrayDeque<>(N * N);
            boolean[] visited01 = new boolean[N * N];

            for (int startBall = 0; startBall < N; startBall++) {
                if (snowmanLocs[startBall])
                    continue;

                pushDistMatrix[startBall][startBall] = 0;
                Arrays.fill(bfsDist01, models.SnowmanConfigurator.UNREACHABLE);
                Arrays.fill(visited01, false);
                deque.clear();

                for (int startChar = 0; startChar < N; startChar++) {
                    if (startChar == startBall || snowmanLocs[startChar])
                        continue;
                    int s = startChar * N + startBall;
                    bfsDist01[s] = 0;
                    deque.addFirst(s);
                }

                while (!deque.isEmpty()) {
                    int state = deque.pollFirst();
                    if (visited01[state])
                        continue;
                    visited01[state] = true;

                    int c = state / N;
                    int b = state % N;
                    int d = bfsDist01[state];

                    // Proposal 1: use adjList instead of O(N) scans
                    for (int ai = 0; ai < adjList[c].length; ai++) {
                        int nextC = adjList[c][ai];
                        if (snowmanLocs[nextC])
                            continue;

                        if (nextC == b) {
                            // PUSH ACTION: find nextB via adjList[b]
                            int pushDir = adjDir[c][ai];
                            int targetNextB = -1;
                            for (int bi = 0; bi < adjList[b].length; bi++) {
                                if (adjDir[b][bi] == pushDir) {
                                    targetNextB = adjList[b][bi];
                                    break;
                                }
                            }

                            if (targetNextB != -1 && !snowmanLocs[targetNextB]) {
                                int nextState = b * N + targetNextB;
                                int newDist = d + 1;
                                if (newDist < bfsDist01[nextState]) {
                                    bfsDist01[nextState] = newDist;
                                    deque.addLast(nextState);

                                    if (newDist < pushDistMatrix[startBall][targetNextB]) {
                                        pushDistMatrix[startBall][targetNextB] = newDist;
                                    }
                                }
                            }
                        } else {
                            // WALK ACTION
                            int nextState = nextC * N + b;
                            if (d < bfsDist01[nextState]) {
                                bfsDist01[nextState] = d;
                                deque.addFirst(nextState);
                            }
                        }
                    }
                }
            }
        }
    } // end SnowmanHeuristic
}