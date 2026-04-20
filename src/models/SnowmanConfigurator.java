package models;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * SnowmanConfigurator – Heuristic push-cost estimator for the Snowman planning
 * problem.
 *
 * ===== KEY DESIGN: Snow Contention =====
 *
 * The dominant source of underestimation in the Snowman heuristic is "snow
 * sharing":
 * when computing costs independently for each ball, two balls are allowed to
 * plan
 * their growth through the SAME snow cell. In the actual problem a snow cell
 * disappears after one ball rolls over it – the second ball must find a
 * different
 * (possibly much more expensive) snow cell.
 *
 * evaluateGrouping() now iterates over ROLE PERMUTATIONS explicitly and,
 * for
 * each (permutation, meeting-location) pair, computes the joint minimum-cost
 * snow
 * assignment forcing distinct snow cells across all balls that need to grow.
 *
 * Two joint-snow helpers are provided:
 *
 * computeJointSnow1_1(locA, locB, tgt) gA=1, gB=1 O(S²)
 * Both role-3 ball (size 2) and role-2 ball (size 1) need exactly 1 snow cell.
 *
 * computeJointSnow2_1(locA, locB, tgt) gA=2, gB=1 O(S³)
 * Role-3 ball needs 2 snow cells, role-2 ball needs 1 snow cell.
 * All three cells must be distinct.
 * This is the MOST COMMON initial-state case (all balls at size 1).
 *
 * With |activeSnowCells| ≤ ~20, S²=400 and S³=8000 are negligible.
 *
 * ===== Other lower-bound improvements =====
 *
 * - obsDist: obstacle-aware push distance (weight 2 for cells occupied by
 * non-grouping balls).
 * - cleanPath BFS: balls with g=0 must not touch snow → cached BFS on snow-free
 * subgraph.
 * - popPenalty: extra pushes when a smaller ball is already at the meeting
 * location
 * and must be popped before the larger one can arrive.
 * - pushDistMatrix: injected from ENHSP after a 0-1 BFS (push=1, walk=0).
 */
public class SnowmanConfigurator {

    // ==================================================================================
    // Constants
    // ==================================================================================

    /** Sentinel for unreachable/infeasible distances. */
    public static final int UNREACHABLE = 100_000;

    public static final double OBSTACLE_PUSH_COST = 2.0;

    // ==================================================================================
    // Fields
    // ==================================================================================

    private final int numLocations;
    private final int numBalls;
    private final int maxSnowmen;

    // All C(numBalls,3) groupings, enumerated once at construction.
    private final List<Grouping> allGroupings;

    // Reused across getBestScore calls.
    private final boolean[] isActiveBalls;
    private final boolean[] usedBalls;

    // ---- Snow environment ----
    private boolean[] currentSnow;
    private final int[] activeSnowCells;
    private int numActiveSnowCells;
    private long snowFingerprint;

    // ---- Push-distance matrix (injected by ENHSP via 0-1 BFS) ----
    private int[][] pushDistMatrix;

    // ---- Transposition cache ----
    private static final int CACHE_SIZE = 1 << 20; // 1 048 576
    private static final long CACHE_MASK = CACHE_SIZE - 1L;
    private final long[] groupingCacheKeys;
    private final double[] groupingCacheLowValues;
    private final double[] groupingCacheUpValues;
    private int[][] cleanPushCache;

    private final int[] cleanDist;
    private final boolean[] cleanVis;
    private final int[] cleanBestTo;
    private final java.util.ArrayDeque<Integer> cleanDq;

    // Pre-allocated arrays for computeJointSnow optimization
    private int[] bestBcost; // bestBcost[k] = pushDist[locB][snow_k] + pushDist[snow_k][tgt]
    private int bestB1val, bestB2val; // top-2 B costs
    private int bestB1idx, bestB2idx; // their snow indices

    // ---- Optimal path tracking (used by ENHSP tie-breaker) ----
    private final Grouping[] currentPath;
    private final Grouping[] optimalGroupings;
    private double globalBestCostLow;
    private double globalBestCostUp;

    // ---- Working arrays for getBestScore ----
    private final double[] precomputedGroupCostsLow;
    private final double[] precomputedGroupCostsUp;
    private final Grouping[] precomputedGroups;
    private final int[] sortedIndices;

    // Preallocated result for evaluateGrouping (zero-allocation on cache miss).
    private final GroupingScore evalResult = new GroupingScore(0.0, 0.0);

    // ==================================================================================
    // Role permutations (static, shared across all instances)
    // ==================================================================================

    // ROLE_PERMS[p] = {iA, iB, iC} where iA/iB/iC ∈ {0,1,2} are the ball indices
    // assigned to roles 3 (base), 2 (middle), 1 (top) respectively.
    private static final int[][] ROLE_PERMS = {
            { 0, 1, 2 }, // b0→role3, b1→role2, b2→role1
            { 0, 2, 1 }, // b0→role3, b2→role2, b1→role1
            { 1, 0, 2 }, // b1→role3, b0→role2, b2→role1
            { 1, 2, 0 }, // b1→role3, b2→role2, b0→role1
            { 2, 0, 1 }, // b2→role3, b0→role2, b1→role1
            { 2, 1, 0 }, // b2→role3, b1→role2, b0→role1
    };

    // ==================================================================================
    // Public API
    // ==================================================================================

    public static class GroupingScore {
        public double low;
        public double up;
        public int bestTgt;
        public int bestBaseId;
        public int bestMidId;
        public int bestHeadId;

        public GroupingScore(double low, double up) {
            this.low = low;
            this.up = up;
            this.bestTgt = -1;
            this.bestBaseId = -1;
            this.bestMidId = -1;
            this.bestHeadId = -1;
        }
    }

    public int getNumActiveSnowCells() {
        return numActiveSnowCells;
    }

    public long getSnowFingerprint() {
        return snowFingerprint;
    }

    public Grouping[] getOptimalGroupings() {
        return optimalGroupings;
    }

    // ==================================================================================
    // Constructor
    // ==================================================================================

    public SnowmanConfigurator(int numLocations, List<Ball> allBalls, int targetSnowmen,
            int[][] distanceMatrix) {
        this.numLocations = numLocations;
        this.numBalls = allBalls.size();
        this.maxSnowmen = targetSnowmen;

        this.activeSnowCells = new int[numLocations];
        this.isActiveBalls = new boolean[this.numBalls];
        this.usedBalls = new boolean[this.numBalls];
        this.currentPath = new Grouping[maxSnowmen];
        this.optimalGroupings = new Grouping[maxSnowmen];

        this.cleanPushCache = new int[numLocations][numLocations];
        for (int i = 0; i < numLocations; i++) {
            Arrays.fill(cleanPushCache[i], -1);
        }

        this.cleanDist = new int[numLocations * numLocations];
        this.cleanVis = new boolean[numLocations * numLocations];
        this.cleanBestTo = new int[numLocations];
        this.cleanDq = new java.util.ArrayDeque<>(numLocations * numLocations);

        // Enumerate all C(n,3) groupings.
        this.allGroupings = new ArrayList<>();
        int n = allBalls.size();
        for (int i = 0; i < n - 2; i++)
            for (int j = i + 1; j < n - 1; j++)
                for (int k = j + 1; k < n; k++)
                    allGroupings.add(new Grouping(allBalls.get(i), allBalls.get(j), allBalls.get(k)));

        int mg = allGroupings.size();
        this.precomputedGroupCostsLow = new double[mg];
        this.precomputedGroupCostsUp = new double[mg];
        this.precomputedGroups = new Grouping[mg];
        this.sortedIndices = new int[mg];

        this.groupingCacheKeys = new long[CACHE_SIZE];
        this.groupingCacheLowValues = new double[CACHE_SIZE];
        this.groupingCacheUpValues = new double[CACHE_SIZE];
        Arrays.fill(this.groupingCacheKeys, -1L);

        // cleanPushCache already initialized at line 180 with fill(-1)
    }

    public void setPushDistMatrix(int[][] m) {
        this.pushDistMatrix = m;
    }

    // Adjacency lists injected from ENHSP for on-demand BFS
    private int[][] adjList;
    private int[][] adjDir;

    public void setAdjacencyLists(int[][] adjList, int[][] adjDir) {
        this.adjList = adjList;
        this.adjDir = adjDir;
    }

    public void clearCache() {
        Arrays.fill(this.groupingCacheKeys, -1L);
    }

    // ==================================================================================
    // Snow environment update
    // ==================================================================================

    public void updateEnvironment(boolean[] currentSnow) {
        this.currentSnow = currentSnow;
        numActiveSnowCells = 0;
        long fp = 0L;
        for (int s = 0; s < numLocations; s++) {
            if (this.currentSnow[s]) {
                activeSnowCells[numActiveSnowCells++] = s;
                fp = fp * 37 + s;
            }
        }

        // Svuotiamo la cache SOLO se l'impronta della neve è cambiata
        if (this.snowFingerprint != fp) {
            this.snowFingerprint = fp;
            for (int i = 0; i < numLocations; i++) {
                Arrays.fill(cleanPushCache[i], -1);
            }
        }
    }

    // ==================================================================================
    // getBestScore — entry point
    // ==================================================================================

    public GroupingScore getBestScore(List<Ball> activeBalls, int targetSnowmen,
            int[] currentSizes, int[] currentLocs) {
        if (activeBalls.size() < 3 * targetSnowmen)
            return new GroupingScore(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

        Arrays.fill(isActiveBalls, false);
        for (Ball b : activeBalls)
            isActiveBalls[b.getIntId()] = true;

        int validCount = 0;

        for (Grouping g : allGroupings) {
            int[] ids = g.getBallIntIds();
            if (!isActiveBalls[ids[0]] || !isActiveBalls[ids[1]] || !isActiveBalls[ids[2]])
                continue;

            Ball b0 = g.getBalls().get(0), b1 = g.getBalls().get(1), b2 = g.getBalls().get(2);

            long hash = computeGroupHash(b0, b1, b2, currentSizes, currentLocs);

            int cacheIdx = (int) (hash & CACHE_MASK);

            double costLow, costUp;
            if (groupingCacheKeys[cacheIdx] == hash) {
                costLow = groupingCacheLowValues[cacheIdx];
                costUp = groupingCacheUpValues[cacheIdx];
            } else {
                evaluateGroupingFast(g, currentSizes, currentLocs);
                costLow = evalResult.low;
                costUp = evalResult.up;
                if (costLow < UNREACHABLE) {
                    groupingCacheKeys[cacheIdx] = hash;
                    groupingCacheLowValues[cacheIdx] = costLow;
                    groupingCacheUpValues[cacheIdx] = costUp;
                }
            }
            precomputedGroups[validCount] = g;
            precomputedGroupCostsLow[validCount] = costLow;
            precomputedGroupCostsUp[validCount] = costUp;
            sortedIndices[validCount] = validCount;
            validCount++;
        }

        if (validCount == 0)
            return new GroupingScore(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

        globalBestCostLow = Double.POSITIVE_INFINITY;
        globalBestCostUp = Double.POSITIVE_INFINITY;
        Arrays.fill(optimalGroupings, null);

        if (targetSnowmen == 1) {
            double minL = Double.POSITIVE_INFINITY, minU = Double.POSITIVE_INFINITY;
            int bi = -1;
            for (int i = 0; i < validCount; i++) {
                double cl = precomputedGroupCostsLow[i], cu = precomputedGroupCostsUp[i];
                if (cl < minL || (cl == minL && cu < minU)) {
                    minL = cl;
                    minU = cu;
                    bi = i;
                }
            }
            if (bi != -1)
                optimalGroupings[0] = precomputedGroups[bi];
            return new GroupingScore(minL, minU);
        }

        primitiveSort(sortedIndices, precomputedGroupCostsLow, precomputedGroupCostsUp, 0, validCount - 1);
        Arrays.fill(usedBalls, false);
        return backtrackRecursive(sortedIndices, validCount, 0, targetSnowmen, usedBalls, 0.0, 0.0, 0);
    }

    // ==================================================================================
    // Backtracking solver
    // ==================================================================================

    private GroupingScore backtrackRecursive(int[] si, int vc, int start, int rem, boolean[] used,
            double cLow, double cUp, int depth) {
        if (rem == 0) {
            if (cLow < globalBestCostLow || (cLow == globalBestCostLow && cUp < globalBestCostUp)) {
                globalBestCostLow = cLow;
                globalBestCostUp = cUp;
                System.arraycopy(currentPath, 0, optimalGroupings, 0, depth);
            }
            return new GroupingScore(cLow, cUp);
        }
        GroupingScore best = new GroupingScore(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        for (int i = start; i < vc; i++) {
            int idx = si[i];
            double cl = precomputedGroupCostsLow[idx], cu = precomputedGroupCostsUp[idx];
            double bnd = cLow + cl * rem;
            if (bnd > best.low)
                break;
            if (bnd > globalBestCostLow)
                break;
            Grouping g = precomputedGroups[idx];
            int[] ids = g.getBallIntIds();
            if (used[ids[0]] || used[ids[1]] || used[ids[2]])
                continue;
            used[ids[0]] = used[ids[1]] = used[ids[2]] = true;
            currentPath[depth] = g;
            GroupingScore res = backtrackRecursive(si, vc, i + 1, rem - 1, used, cLow + cl, cUp + cu, depth + 1);
            if (res.low < best.low || (res.low == best.low && res.up < best.up)) {
                best.low = res.low;
                best.up = res.up;
            }
            used[ids[0]] = used[ids[1]] = used[ids[2]] = false;
        }
        return best;
    }

    // ==================================================================================
    // evaluateGrouping — CORE: explicit permutations + joint snow assignment
    // ==================================================================================
    // ==================================================================================
    // computeJointSnow — FULLY DYNAMIC snow detour computation
    // ==================================================================================
    //
    // Uses activeSnowCells (updated every call via updateEnvironment) for ALL
    // cases.
    // Uses activeSnowCells to keep detours updated after snowman completion.

    private double computeJointSnow(int locA, int gA, int locB, int gB, int tgt) {
        // (0,0): no snow needed — A (sz=3) can cross snow, B (sz=2, g=0) must NOT!
        if (gA == 0 && gB == 0)
            return pushDistMatrix[locA][tgt] + computeCleanPushDist(locB, tgt);

        int S = numActiveSnowCells;
        if (gA + gB > S)
            return UNREACHABLE;

        // (1,0): A needs 1 snow, B needs 0 — iterate activeSnowCells for A
        if (gA == 1 && gB == 0) {
            double best = UNREACHABLE;
            for (int i = 0; i < S; i++) {
                int s = activeSnowCells[i];
                int d = pushDistMatrix[locA][s] + pushDistMatrix[s][tgt];
                if (d < best)
                    best = d;
            }
            return best + computeCleanPushDist(locB, tgt);
        }
        // (0,1): symmetric
        if (gA == 0 && gB == 1) {
            double best = UNREACHABLE;
            for (int i = 0; i < S; i++) {
                int s = activeSnowCells[i];
                int d = pushDistMatrix[locB][s] + pushDistMatrix[s][tgt];
                if (d < best)
                    best = d;
            }
            return pushDistMatrix[locA][tgt] + best;
        }

        // (2,0): A needs 2 snow cells, B needs 0
        if (gA == 2 && gB == 0) {
            double best = UNREACHABLE;
            for (int i = 0; i < S; i++) {
                int s1 = activeSnowCells[i];
                int d1 = pushDistMatrix[locA][s1];
                if (d1 >= UNREACHABLE)
                    continue;
                for (int j = 0; j < S; j++) {
                    if (i == j)
                        continue;
                    int s2 = activeSnowCells[j];
                    int d = d1 + pushDistMatrix[s1][s2] + pushDistMatrix[s2][tgt];
                    if (d < best)
                        best = d;
                }
            }
            return best + computeCleanPushDist(locB, tgt);
        }
        // (0,2): symmetric
        if (gA == 0 && gB == 2) {
            double best = UNREACHABLE;
            for (int i = 0; i < S; i++) {
                int s1 = activeSnowCells[i];
                int d1 = pushDistMatrix[locB][s1];
                if (d1 >= UNREACHABLE)
                    continue;
                for (int j = 0; j < S; j++) {
                    if (i == j)
                        continue;
                    int s2 = activeSnowCells[j];
                    int d = d1 + pushDistMatrix[s1][s2] + pushDistMatrix[s2][tgt];
                    if (d < best)
                        best = d;
                }
            }
            return pushDistMatrix[locA][tgt] + best;
        }

        double bestCost = UNREACHABLE;

        // --- OPTIMIZATION: Precompute B's best-via-1-snow costs ---
        // For each snow cell k, compute costB[k] = pushDist[locB][snow_k] +
        // pushDist[snow_k][tgt]
        // Also track the top-2 best values so we can exclude specific indices in O(1).
        if (bestBcost == null || bestBcost.length < S)
            bestBcost = new int[S];
        bestB1val = UNREACHABLE;
        bestB2val = UNREACHABLE;
        bestB1idx = -1;
        bestB2idx = -1;
        for (int k = 0; k < S; k++) {
            int sk = activeSnowCells[k];
            int c = pushDistMatrix[locB][sk] + pushDistMatrix[sk][tgt];
            bestBcost[k] = c;
            if (c < bestB1val) {
                bestB2val = bestB1val;
                bestB2idx = bestB1idx;
                bestB1val = c;
                bestB1idx = k;
            } else if (c < bestB2val) {
                bestB2val = c;
                bestB2idx = k;
            }
        }

        // (1,1): contention — A and B each need 1 distinct snow cell
        if (gA == 1 && gB == 1) {
            for (int i = 0; i < S; i++) {
                int s1 = activeSnowCells[i];
                int cA = pushDistMatrix[locA][s1] + pushDistMatrix[s1][tgt];
                if (cA >= bestCost)
                    continue;
                // Best B excluding snow index i
                int cB = (i != bestB1idx) ? bestB1val : bestB2val;
                if (cA + cB < bestCost)
                    bestCost = cA + cB;
            }
        }
        // (2,1): A needs 2 distinct, B needs 1 distinct from A's
        else if (gA == 2 && gB == 1) {
            for (int i = 0; i < S; i++) {
                int s1 = activeSnowCells[i];
                int dA1 = pushDistMatrix[locA][s1];
                if (dA1 >= UNREACHABLE)
                    continue;
                for (int j = 0; j < S; j++) {
                    if (i == j)
                        continue;
                    int s2 = activeSnowCells[j];
                    int dA12 = pushDistMatrix[s1][s2];
                    if (dA12 >= UNREACHABLE)
                        continue;
                    int dA2T = pushDistMatrix[s2][tgt];
                    if (dA2T >= UNREACHABLE)
                        continue;
                    int cA = dA1 + dA12 + dA2T;
                    if (cA >= bestCost)
                        continue;
                    // Best B excluding snow indices i and j
                    int cB;
                    if (bestB1idx != i && bestB1idx != j)
                        cB = bestB1val;
                    else if (bestB2idx != i && bestB2idx != j)
                        cB = bestB2val;
                    else {
                        // top-2 both excluded by A's choices — scan remaining cells
                        cB = UNREACHABLE;
                        for (int kk = 0; kk < S; kk++) {
                            if (kk != i && kk != j && bestBcost[kk] < cB)
                                cB = bestBcost[kk];
                        }
                    }
                    if (cA + cB < bestCost)
                        bestCost = cA + cB;
                }
            }
        }
        // (1,2): symmetric
        else if (gA == 1 && gB == 2) {
            return computeJointSnow(locB, gB, locA, gA, tgt);
        }
        return bestCost;
    }

    // ==================================================================================
    // computeCleanPushDist — on-demand BFS avoiding snow cells
    // ==================================================================================
    //
    // Balls that should NOT grow (g=0, sz<3) must travel on snow-free
    // paths. This does a lazy, targeted 0-1 BFS from 'from' avoiding snow cells,
    // returning the minimum number of pushes to reach 'to'.
    // Only called when needed (g=0 and sz<3), so the cost is amortized.

    private int computeCleanPushDist(int from, int to) {
        if (from == to)
            return 0;
        if (adjList == null)
            return pushDistMatrix[from][to];

        // Ritorno istantaneo se la BFS da 'from' è già stata calcolata
        if (cleanPushCache[from][0] != -1) {
            int res = cleanPushCache[from][to];
            return (res >= UNREACHABLE && pushDistMatrix[from][to] < UNREACHABLE)
                    ? pushDistMatrix[from][to]
                    : res;
        }

        int N = numLocations;
        // Uso delle strutture pre-allocate azzerandole
        Arrays.fill(cleanDist, UNREACHABLE);
        Arrays.fill(cleanVis, false);
        Arrays.fill(cleanBestTo, UNREACHABLE);
        cleanDq.clear();

        // Multi-source
        for (int c = 0; c < N; c++) {
            if (c == from)
                continue;
            int s = c * N + from;
            cleanDist[s] = 0;
            cleanDq.addFirst(s);
        }
        cleanBestTo[from] = 0;

        while (!cleanDq.isEmpty()) {
            int state = cleanDq.pollFirst();
            if (cleanVis[state])
                continue;
            cleanVis[state] = true;

            int c = state / N;
            int b = state % N;
            int d = cleanDist[state];

            for (int ai = 0; ai < adjList[c].length; ai++) {
                int nextC = adjList[c][ai];

                if (nextC == b) {
                    // PUSH: la direzione della palla
                    int pushDir = adjDir[c][ai];
                    int targetNextB = -1;
                    for (int bi = 0; bi < adjList[b].length; bi++) {
                        if (adjDir[b][bi] == pushDir) {
                            targetNextB = adjList[b][bi];
                            break;
                        }
                    }
                    // LA PALLA (NON IL PERSONAGGIO) NON DEVE FINIRE SULLA NEVE
                    if (targetNextB != -1 && !(currentSnow != null && currentSnow[targetNextB])) {
                        int nextState = b * N + targetNextB;
                        int newDist = d + 1;
                        if (newDist < cleanDist[nextState]) {
                            cleanDist[nextState] = newDist;
                            cleanDq.addLast(nextState);
                            if (newDist < cleanBestTo[targetNextB]) {
                                cleanBestTo[targetNextB] = newDist;
                            }
                        }
                    }
                } else {
                    // WALK: Il personaggio cammina liberamente (anche sulla neve!)
                    int nextState = nextC * N + b;
                    if (d < cleanDist[nextState]) {
                        cleanDist[nextState] = d;
                        cleanDq.addFirst(nextState);
                    }
                }
            }
        }

        // Salviamo in blocco tutte le distanze calcolate da 'from' verso ogni 'to'
        for (int i = 0; i < N; i++) {
            cleanPushCache[from][i] = cleanBestTo[i];
        }

        int res = cleanPushCache[from][to];
        return (res >= UNREACHABLE && pushDistMatrix[from][to] < UNREACHABLE)
                ? pushDistMatrix[from][to]
                : res;
    }

    public GroupingScore getOptimalRolesAndTarget(Grouping g, int[] currentSizes, int[] currentLocs) {
        evaluateGroupingFast(g, currentSizes, currentLocs);
        return evalResult;
    }

    private void evaluateGroupingFast(Grouping group, int[] currentSizes, int[] currentLocs) {
        int id0 = group.getBallIntIds()[0];
        int id1 = group.getBallIntIds()[1];
        int id2 = group.getBallIntIds()[2];

        int loc0 = currentLocs[id0];
        int loc1 = currentLocs[id1];
        int loc2 = currentLocs[id2];

        int[] ballLocs = { loc0, loc1, loc2 };
        int[] ballSzs = { currentSizes[id0], currentSizes[id1], currentSizes[id2] };

        double bestLow = Double.POSITIVE_INFINITY;
        double bestUp = Double.POSITIVE_INFINITY;
        int bestTarget = -1;
        int bestBase = -1, bestMid = -1, bestHead = -1;

        for (int tgt = 0; tgt < numLocations; tgt++) {
            // --- Target pruning (admissible lower bound) ---
            // The true cost for this target is at least the sum of the THREE smallest
            // individual push distances (one per ball, relaxing role assignment and snow).
            // We sort the three values cheaply and sum them: only if that sum already
            // meets or beats bestLow can we prune safely.
            int d0 = pushDistMatrix[loc0][tgt];
            int d1 = pushDistMatrix[loc1][tgt];
            int d2 = pushDistMatrix[loc2][tgt];
            // Inline 3-element sort to get d0 ≤ d1 ≤ d2
            if (d0 > d1) {
                int t = d0;
                d0 = d1;
                d1 = t;
            }
            if (d1 > d2) {
                int t = d1;
                d1 = d2;
                d2 = t;
            }
            if (d0 > d1) {
                int t = d0;
                d0 = d1;
                d1 = t;
            }
            // d0+d1+d2 is an admissible lower bound on the total cost for any assignment
            if ((long) d0 + d1 + d2 >= (long) bestLow)
                continue; // This target cannot beat the current best

            for (int[] perm : ROLE_PERMS) {
                int iA = perm[0], iB = perm[1], iC = perm[2];
                int locA = ballLocs[iA], szA = ballSzs[iA];
                int locB = ballLocs[iB], szB = ballSzs[iB];
                int locC = ballLocs[iC], szC = ballSzs[iC];

                if (szA > 3 || szB > 2 || szC > 1)
                    continue;

                int gA = 3 - szA;
                int gB = 2 - szB;

                double jointCostAB = computeJointSnow(locA, gA, locB, gB, tgt);
                if (jointCostAB >= UNREACHABLE)
                    continue;

                double costC = computeCleanPushDist(locC, tgt);
                if (costC >= UNREACHABLE)
                    continue;

                // Compute cleanFallbackPenalty BEFORE the early exit so
                // that totalUp is always correctly penalised, even when jointCostAB+costC
                // already beats bestLow. The penalty only affects h_up (tie-breaking),
                // never h_balls, so admissibility is preserved.
                double cleanFallbackPenalty = 0.0;
                if (gB == 0 && pushDistMatrix[locB][tgt] < UNREACHABLE) {
                    int cleanB = computeCleanPushDist(locB, tgt);
                    if (cleanB > pushDistMatrix[locB][tgt])
                        cleanFallbackPenalty += 1000.0;
                }
                // Head ball (locC) always uses clean path
                if (pushDistMatrix[locC][tgt] < UNREACHABLE) {
                    int cleanC = computeCleanPushDist(locC, tgt);
                    if (cleanC > pushDistMatrix[locC][tgt])
                        cleanFallbackPenalty += 1000.0;
                }

                // Early exit: if this permutation's base cost alone can't beat bestLow,
                // skip the heavier penalty computation — but only after the penalty is set.
                if (jointCostAB + costC >= bestLow)
                    continue;

                double penalty = computePopPenalty(locA, locB, locC, tgt);
                double totalLow = jointCostAB + costC + penalty; // admissible lower bound

                double obstaclePenalty = 0.0;

                for (int bOther = 0; bOther < numBalls; bOther++) {
                    if (!isActiveBalls[bOther])
                        continue;
                    if (bOther == id0 || bOther == id1 || bOther == id2)
                        continue;
                    int locOther = currentLocs[bOther];

                    if (jointCostAB < UNREACHABLE && pushDistMatrix[locA][locOther]
                            + pushDistMatrix[locOther][tgt] == pushDistMatrix[locA][tgt])
                        obstaclePenalty += OBSTACLE_PUSH_COST;
                    if (jointCostAB < UNREACHABLE && pushDistMatrix[locB][locOther]
                            + pushDistMatrix[locOther][tgt] == pushDistMatrix[locB][tgt])
                        obstaclePenalty += OBSTACLE_PUSH_COST;
                    if (costC < UNREACHABLE && pushDistMatrix[locC][locOther]
                            + pushDistMatrix[locOther][tgt] == pushDistMatrix[locC][tgt])
                        obstaclePenalty += OBSTACLE_PUSH_COST;
                }
                double totalUp = totalLow + obstaclePenalty + cleanFallbackPenalty;

                if (totalLow < bestLow || (totalLow == bestLow && totalUp < bestUp)) {
                    bestLow = totalLow;
                    bestUp = totalUp;
                    bestTarget = tgt;
                    bestBase = group.getBallIntIds()[iA];
                    bestMid = group.getBallIntIds()[iB];
                    bestHead = group.getBallIntIds()[iC];
                }
            }
        }

        evalResult.low = (bestLow >= UNREACHABLE) ? UNREACHABLE : bestLow;
        evalResult.up = (bestUp >= UNREACHABLE) ? UNREACHABLE : bestUp;
        evalResult.bestTgt = bestTarget;
        evalResult.bestBaseId = bestBase;
        evalResult.bestMidId = bestMid;
        evalResult.bestHeadId = bestHead;
    }

    // ==================================================================================
    // Pop penalty
    // ==================================================================================

    /**
     * Extra pushes needed when balls are already at the target in the wrong stack
     * order.
     * 
     * @param loc3    current location of the ball assigned to role 3 (base)
     * @param loc2    current location of the ball assigned to role 2 (middle)
     * @param loc1    current location of the ball assigned to role 1 (top)
     * @param targetL the meeting location
     */
    private double computePopPenalty(int loc3, int loc2, int loc1, int targetL) {
        double p = 0.0;
        boolean baseMissing = (loc3 != targetL);
        boolean midMissing = (loc2 != targetL);
        // Middle ball at target but base not yet arrived: must pop middle first.
        if (!midMissing && baseMissing)
            p += 2.0;
        // Top ball at target but base or middle missing: must pop top first.
        if (loc1 == targetL && (baseMissing || midMissing))
            p += 2.0;
        return p;
    }

    // ==================================================================================
    // Hash and sort helpers
    // ==================================================================================

    private long computeGroupHash(Ball b0, Ball b1, Ball b2, int[] sz, int[] lc) {
        long h = snowFingerprint;
        // Hash delle palle del gruppo (con taglia)
        h ^= (lc[b0.getIntId()] * 131L + sz[b0.getIntId()]) * 1_000_000_007L;
        h ^= (lc[b1.getIntId()] * 131L + sz[b1.getIntId()]) * 1_000_000_009L;
        h ^= (lc[b2.getIntId()] * 131L + sz[b2.getIntId()]) * 1_000_000_021L;

        // Include active ball positions OUTSIDE the grouping for obstaclePenalty
        // invalidation. The 3 grouping balls are already hashed above with distinct
        // prime multipliers — including them again would only reduce cache hit rate.
        int id0 = b0.getIntId(), id1 = b1.getIntId(), id2 = b2.getIntId();
        for (int i = 0; i < numBalls; i++) {
            if (isActiveBalls[i] && i != id0 && i != id1 && i != id2) {
                h = h * 31L + lc[i];
            }
        }

        h ^= h >>> 33;
        h *= 0xff51afd7ed558ccdL;
        h ^= h >>> 33;
        h *= 0xc4ceb9fe1a85ec53L;
        h ^= h >>> 33;
        return h;
    }

    private void primitiveSort(int[] idx, double[] kl, double[] ku, int lo, int hi) {
        int len = hi - lo + 1;
        if (len < 2)
            return;
        if (len < 32) {
            for (int i = lo + 1; i <= hi; i++) {
                int ki = idx[i];
                double kli = kl[ki], kui = ku[ki];
                int j = i - 1;
                while (j >= lo && (kl[idx[j]] > kli || (kl[idx[j]] == kli && ku[idx[j]] > kui))) {
                    idx[j + 1] = idx[j];
                    j--;
                }
                idx[j + 1] = ki;
            }
            return;
        }
        int pi = lo + (hi - lo) / 2;
        double pl = kl[idx[pi]], pu = ku[idx[pi]];
        int i = lo, j = hi;
        while (i <= j) {
            while (kl[idx[i]] < pl || (kl[idx[i]] == pl && ku[idx[i]] < pu))
                i++;
            while (kl[idx[j]] > pl || (kl[idx[j]] == pl && ku[idx[j]] > pu))
                j--;
            if (i <= j) {
                int t = idx[i];
                idx[i] = idx[j];
                idx[j] = t;
                i++;
                j--;
            }
        }
        primitiveSort(idx, kl, ku, lo, j);
        primitiveSort(idx, kl, ku, i, hi);
    }
}