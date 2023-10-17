import java.io.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;

import org.apache.spark.mllib.linalg.Vector;
import org.apache.spark.mllib.linalg.Vectors;
import scala.Tuple2;


public class G044HW2 {

    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // Input reading methods (provided by professor)
    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static Vector strToVector(String str) {
        String[] tokens = str.split(",");
        double[] data = new double[tokens.length];
        for (int i = 0; i < tokens.length; i++) {
            data[i] = Double.parseDouble(tokens[i]);
        }
        return Vectors.dense(data);
    }

    public static ArrayList<Vector> readVectorsSeq(String filename) throws IOException {
        if (Files.isDirectory(Paths.get(filename))) {
            throw new IllegalArgumentException("readVectorsSeq is meant to read a single file.");
        }
        ArrayList<Vector> result = new ArrayList<>();
        Files.lines(Paths.get(filename)).map(str -> strToVector(str)).forEach(e -> result.add(e));
        return result;
    }

    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // Utility function to precompute the distances of a point set
    // used in SeqWeightedOutliers(P,W,k,z,alpha) function.
    //
    // NOTICE THAT: the computed matrix is an upper triangular matrix
    // in order to save more space; therefore it's necessary an index
    // check every time we need to access to the matrix.
    // In fact: d(x,y) == d(y,x) and d(x,x) == 0, for each point x,y
    // that belong to a point set.
    // Instead, to save computational time, it's sufficient to fill
    // the whole matrix, so to avoid the multiple checks when it
    // is needed to access to the matrix.
    //
    // Because the algorithm is still time efficient with the largest
    // dataset, we have decided to save space and fill up only half
    // of the distances matrix, which is relevant (the memory space)
    // for our kind of work.
    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static double[][] precomputeDistances(ArrayList<Vector> pointSet, boolean half_matrix) {
        double[][] distances = new double[pointSet.size()][pointSet.size()];
        //computing distances among the input set points
        for (int i = 0; i < pointSet.size(); i++) {
            distances[i][i] = 0.0;
            for (int j = i + 1; j < pointSet.size(); j++) {
                distances[i][j] = Math.sqrt(Vectors.sqdist(pointSet.get(i), pointSet.get(j)));
                if(!half_matrix){
                    distances[j][i] = distances[i][j];
                }
            }
        }

        return distances;
    }

    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // 01. SeqWeightedOutliers(P,W,k,z,alpha) function
    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static ArrayList<Vector> SeqWeightedOutliers(ArrayList<Vector> P,
                                                        ArrayList<Long> W,
                                                        int k, int z,
                                                        double alpha) {

        //precompute the distances among the whole point set: fill up only half matrix
        boolean half_matrix = false;
        double[][] distances = precomputeDistances(P, half_matrix);

        //looking for min distance among first k+z+1 points
        double r = Double.MAX_VALUE;
        for (int i = 0; i < (k + z); i++) {
            for (int j = i + 1; j < (k + z + 1); j++) {
                //check indices validity for matrix distances and retrieve correct distance value
                double distance = distances[i][j];
                if(half_matrix && i > j)    distance = distances[j][i];
                //update r
                if (distance < r) {
                    r = distance;
                }
            }
        }
        r /= 2;
        int n_guesses = 1;
        System.out.println("Initial guess = " + r);

        while (true) {
            //set Z stores which indices of P can be considered
            boolean[] Z = new boolean[P.size()];
            for (int p = 0; p < P.size(); p++) {
                Z[p] = true;
            }
            //set S to empty
            ArrayList<Vector> S = new ArrayList<>(0);
            //compute W_z
            long W_z = 0L;
            for (long w : W) {
                W_z += w;
            }
            //compute centers set S
            while (S.size() < k && W_z > 0) {
                long max = 0L;
                int newcenter = -1;
                for (int x = 0; x < P.size(); x++) {
                    long ball_weight = 0L;
                    //computing the weight of the ball in Z centered in x with radius (1+2*aplha)*r
                    double ball_radius = (1 + 2 * alpha) * r;
                    for (int y = 0; y < P.size(); y++) {
                        if (Z[y]) {
                            //check indices validity for matrix distances and retrieve correct distance value
                            double distance = distances[x][y];
                            if(half_matrix && x > y)    distance = distances[y][x];
                            //update weight of the ball
                            if(distance <= ball_radius){
                                ball_weight += W.get(y);
                            }
                        }
                    }
                    //update ball_weight if heavier than the already seen ones
                    if (ball_weight > max) {
                        max = ball_weight;
                        newcenter = x;
                    }
                }
                //add newcenter to set S
                S.add(P.get(newcenter));
                //for each point y that belongs to the ball in Z centered in newcenter with radius (3 + 4*alpha)*r)
                double ball_radius = (3 + 4 * alpha) * r;
                for (int y = 0; y < P.size(); y++) {
                    if (Z[y]) {
                        //check indices validity for matrix distances and retrieve correct distance value
                        double distance = distances[newcenter][y];
                        if(half_matrix && newcenter > y)    distance = distances[y][newcenter];
                        //update total weight of Z and Z
                        if(distance <= ball_radius){
                            W_z -= W.get(y);
                            Z[y] = false;
                        }
                    }
                }
            }
            //return S if we have not more than then allowed outliers, otherwise increase r
            if (W_z <= z) {
                System.out.println("Final guess = " + r);
                System.out.println("Number of guesses = " + n_guesses);
                return S;
            } else {
                r *= 2;
                n_guesses++;
            }
        }
    }

    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // 02. ComputeObjective(P,S,z) function
    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static double ComputeObjective(ArrayList<Vector> P, ArrayList<Vector> S, int z) {
        //compute the distance of each x from S
        ArrayList<Double> distances_from_S = new ArrayList<>(P.size());
        for (int x = 0; x < P.size(); x++) {
            double min = Double.MAX_VALUE;
            for (int y = 0; y < S.size(); y++) {
                double distance = Math.sqrt(Vectors.sqdist(P.get(x), S.get(y)));
                if (distance < min) {
                    min = distance;
                }
            }
            distances_from_S.add(x, min);
        }
        //return the max distance from S among points in P
        distances_from_S.sort(Comparator.reverseOrder());
        return distances_from_S.get(z);
    }

    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // 03. Method main()
    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static void main(String[] args) throws IOException {

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // CHECKING NUMBER OF CMD LINE PARAMETERS
        // Parameters are: path_to_point_set_txt_file, number_centers, number_outliers
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        if (args.length != 3) {
            throw new IllegalArgumentException("USAGE: <path_to_point_set_txt_file> <number_centers> <number_outliers>");
        }

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // INPUT READING & ROWS PRINTING
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        // SETTING INPUT VARIABLES RECEIVED BY CLI INPUT

        // Input correctness check and variable assignment
        int k, z;
        String file_path;
        try {
            file_path = args[0];
            k = Integer.parseInt(args[1]);
            z = Integer.parseInt(args[2]);
        } catch (Exception e) {
            throw new IllegalArgumentException("USAGE: <path_to_point_set_txt_file> <number_centers> <number_outliers>");
        }

        //point set input reading
        ArrayList<Vector> inputPoints = readVectorsSeq(file_path);
        //weights setting
        ArrayList<Long> weights = new ArrayList<>(Collections.nCopies(inputPoints.size(), 1L));

        System.out.println("Input size n = " + inputPoints.size());
        System.out.println("Number of centers k = " + k);
        System.out.println("Number of outliers z = " + z);

        //solution S computing and computation timing of S
        long start = System.currentTimeMillis();
        ArrayList<Vector> solution = SeqWeightedOutliers(inputPoints, weights, k, z, 0.0);
        long end = System.currentTimeMillis();
        //objective function computing
        double objective = ComputeObjective(inputPoints, solution, z);

        System.out.println("Objective function = " + objective);
        System.out.println("Time of SeqWeightedOutliers = " + (end - start));

    }

}
