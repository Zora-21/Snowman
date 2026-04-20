
/* 
 * Copyright (C) 2015-2017, Enrico Scala, contact: enricos83@gmail.com
 * Modified for Snowman Extension, 2026
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
 * 
 */

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.concurrent.TimeUnit;
import planners.ENHSP;

public class Main {

    /*
     * Per eseguire da linea di comando con argomenti arbitrari:
     * public static void main(String[] args) throws Exception {
     * ENHSP p = new ENHSP(false);
     * p.parseInput(args);
     * p.configurePlanner();
     * p.parsingDomainAndProblem(args);
     * p.planning();
     * }
     */

    public static void main(String[] args) throws Exception {
        if (args.length > 0) {
            ENHSP p = new ENHSP(false);
            p.parseInput(args);
            p.configurePlanner();
            p.parsingDomainAndProblem(args);
            p.planning();
        } else {
            String baseDir = System.getProperty("user.dir");

            String[] listFiles = {
                    baseDir + "/Project_dh/name/name_one.txt",
                    baseDir + "/Project_dh/name/name_two.txt",
                    baseDir + "/Project_dh/name/name_three.txt"
            };

            String[] problemDirs = {
                    baseDir + "/Project_dh/problem/1-snowman/",
                    baseDir + "/Project_dh/problem/2-snowmen/",
                    baseDir + "/Project_dh/problem/3-snowmen/"
            };

            String[] typologies = {
                    "1-snowman",
                    "2-snowmen",
                    "3-snowmen"
            };

            String[] domainFiles = {
                    baseDir + "/Project_dh/domain/Domain_one.pddl",
                    baseDir + "/Project_dh/domain/Domain_two.pddl",
                    baseDir + "/Project_dh/domain/Domain_three.pddl"
            };

            for (int i = 0; i < listFiles.length; i++) {
                String listFile = listFiles[i];
                String domain = domainFiles[i];
                String basePath = problemDirs[i];
                String planPath = baseDir + "/Project_dh/plan";

                File lf = new File(listFile);
                if (!lf.exists()) {
                    System.out.println("Lista non trovata: " + listFile);
                    continue;
                }

                List<String> inputNames;
                try {
                    inputNames = Files.readAllLines(Paths.get(listFile));
                } catch (Exception e) {
                    System.out.println("Errore nella lettura di " + listFile);
                    continue;
                }

                for (String name : inputNames) {
                    name = name.trim();
                    if (name.isEmpty())
                        continue;

                    System.out.println("--- Inizio elaborazione per: " + name + " (" + basePath + ") ---");

                    String problem = basePath + name + "_problem.pddl";
                    String typology = typologies[i];
                    String outputDir = planPath + "/" + typology + "/";
                    new File(outputDir).mkdirs();
                    String outputFile = outputDir + name + "_plan.txt";

                    if (!new File(problem).exists()) {
                        System.out
                                .println("Errore: File del problema non trovato: " + problem + ". Salto al prossimo.");
                        System.out.println("-----------------------------------\n");
                        continue;
                    }

                    try {
                        // Lanciamo un nuovo processo Java per ogni problema per garantire l'isolamento
                        // della memoria.
                        // Questo risolve i problemi di leak di stato statico della libreria ppmajal.
                        ProcessBuilder pb = new ProcessBuilder(
                                "java",
                                "-cp", "bin:lib/enhsp-20.jar",
                                "Main",
                                "-o", domain,
                                "-f", problem, "-planner", "snowman");

                        // Reindirizziamo l'output direttamente nel file di piano
                        pb.redirectOutput(new File(outputFile));
                        pb.redirectErrorStream(true); // Cattura anche eventuali errori nel file

                        Process process = pb.start();

                        // TIMER
                        // -------------------------------------------------------------------------------------
                        int TIME = 300;
                        // -------------------------------------------------------------------------------------

                        boolean finished = process.waitFor(TIME, TimeUnit.SECONDS);

                        if (finished) {
                            int exitCode = process.exitValue();
                            if (exitCode == 0) {
                                System.out.println("Output salvato con successo in " + outputFile);
                            } else {
                                System.out.println("Il processo è terminato con un errore (exit code: " + exitCode
                                        + ") per " + name);
                            }
                        } else {
                            // Timeout raggiunto! Killiamo il processo
                            process.destroyForcibly();
                            System.out
                                    .println("TIMEOUT: " + name + " ha superato i " + TIME
                                            + " secondi. Passaggio al prossimo.");
                        }

                    } catch (Exception e) {
                        System.out.println(
                                "Si è verificato un errore imprevisto durante l'avvio del processo per " + name + ": "
                                        + e.getMessage());
                        e.printStackTrace();
                    }

                    System.out.println("--- Elaborazione per " + name + " completata --- \n");
                }
            }
        }
    }

}
