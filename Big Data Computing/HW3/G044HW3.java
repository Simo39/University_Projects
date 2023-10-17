import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.spark.SparkConf;
import org.apache.spark.api.java.JavaDoubleRDD;
import org.apache.spark.api.java.JavaPairRDD;
import org.apache.spark.api.java.JavaRDD;
import org.apache.spark.api.java.JavaSparkContext;
import org.apache.spark.mllib.linalg.BLAS;
import org.apache.spark.mllib.linalg.Vector;
import org.apache.spark.mllib.linalg.Vectors;
import scala.Tuple2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class G044HW3 {
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// MAIN PROGRAM
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static void main(String[] args) throws Exception {

        if (args.length != 4) {
            throw new IllegalArgumentException("USAGE: filepath k z L");
        }

        // ----- Initialize variables
        String filename = args[0];
        int k = Integer.parseInt(args[1]);
        int z = Integer.parseInt(args[2]);
        int L = Integer.parseInt(args[3]);
        long start, end; // variables for time measurements

        // ----- Set Spark Configuration
        Logger.getLogger("org").setLevel(Level.OFF);
        Logger.getLogger("akka").setLevel(Level.OFF);
        SparkConf conf = new SparkConf(true).setAppName("MR k-center with outliers");
        JavaSparkContext sc = new JavaSparkContext(conf);
        sc.setLogLevel("WARN");

        // ----- Read points from file
        start = System.currentTimeMillis();
        JavaRDD<Vector> inputPoints = sc.textFile(args[0], L)
                .map(x -> strToVector(x))
                .repartition(L)
                .cache();
        long N = inputPoints.count();
        end = System.currentTimeMillis();

        // ----- Print input parameters
        System.out.println("File : " + filename);
        System.out.println("Number of points N = " + N);
        System.out.println("Number of centers k = " + k);
        System.out.println("Number of outliers z = " + z);
        System.out.println("Number of partitions L = " + L);
        System.out.println("Time to read from file: " + (end - start) + " ms");

        // ---- Solve the problem
        ArrayList<Vector> solution = MR_kCenterOutliers(inputPoints, k, z, L);

        // ---- Compute the value of the objective function
        start = System.currentTimeMillis();
        double objective = computeObjective(inputPoints, solution, z);
        end = System.currentTimeMillis();
        System.out.println("Objective function = " + objective);
        System.out.println("Time to compute objective function: " + (end - start) + " ms");

    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// AUXILIARY METHODS
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method strToVector: input reading
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static Vector strToVector(String str) {
        String[] tokens = str.split(",");
        double[] data = new double[tokens.length];
        for (int i = 0; i < tokens.length; i++) {
            data[i] = Double.parseDouble(tokens[i]);
        }
        return Vectors.dense(data);
    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method euclidean: distance function
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static double euclidean(Vector a, Vector b) {
        return Math.sqrt(Vectors.sqdist(a, b));
    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method MR_kCenterOutliers: MR algorithm for k-center with outliers
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static ArrayList<Vector> MR_kCenterOutliers(JavaRDD<Vector> points, int k, int z, int L) {

        //------------- ROUND 1 ---------------------------

        long start1 = System.currentTimeMillis();
        JavaRDD<Tuple2<Vector, Long>> coreset = points.mapPartitions(x ->
        {
            ArrayList<Vector> partition = new ArrayList<>();
            while (x.hasNext()) partition.add(x.next());
            ArrayList<Vector> centers = kCenterFFT(partition, k + z + 1);
            ArrayList<Long> weights = computeWeights(partition, centers);
            ArrayList<Tuple2<Vector, Long>> c_w = new ArrayList<>();
            for (int i = 0; i < centers.size(); ++i) {
                Tuple2<Vector, Long> entry = new Tuple2<>(centers.get(i), weights.get(i));
                c_w.add(i, entry);
            }
            return c_w.iterator();
        }); // END OF ROUND 1

        //------------- ROUND 2 ---------------------------

        ArrayList<Tuple2<Vector, Long>> elems = new ArrayList<>();
        elems.addAll(coreset.collect());
        // elems.addAll(coreset.distinct().collect());
        long end1 = System.currentTimeMillis();

        // ****** ADD YOUR CODE
        // ****** Compute the final solution (run SeqWeightedOutliers with alpha=2)
        // ****** Measure and print times taken by Round 1 and Round 2, separately
        // ****** Return the final solution
        //

        long start2 = System.currentTimeMillis();
        ArrayList<Vector> P = new ArrayList<>();
        ArrayList<Long> W = new ArrayList<>();
        for (Tuple2<Vector, Long> elem : elems) {
            P.add(elem._1());
            W.add(elem._2());
        }

        // ****** Compute the final solution (run SeqWeightedOutliers with alpha=2)
        ArrayList<Vector> solution = SeqWeightedOutliers(P, W, k, z, 2.0);

        // ****** Measure and print times taken by Round 1 and Round 2, separately
        long end2 = System.currentTimeMillis();
        System.out.println("Time Round 1: " + (end1 - start1) + " ms");
        System.out.println("Time Round 2: " + (end2 - start2) + " ms");

        // ****** Return the final solution
        return solution;
    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method kCenterFFT: Farthest-First Traversal
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static ArrayList<Vector> kCenterFFT(ArrayList<Vector> points, int k) {

        final int n = points.size();
        double[] minDistances = new double[n];
        Arrays.fill(minDistances, Double.POSITIVE_INFINITY);

        ArrayList<Vector> centers = new ArrayList<>(k);

        Vector lastCenter = points.get(0);
        centers.add(lastCenter);
        double radius = 0;

        for (int iter = 1; iter < k; iter++) {
            int maxIdx = 0;
            double maxDist = 0;

            for (int i = 0; i < n; i++) {
                double d = euclidean(points.get(i), lastCenter);
                if (d < minDistances[i]) {
                    minDistances[i] = d;
                }

                if (minDistances[i] > maxDist) {
                    maxDist = minDistances[i];
                    maxIdx = i;
                }
            }

            lastCenter = points.get(maxIdx);
            centers.add(lastCenter);
        }
        return centers;
    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method computeWeights: compute weights of coreset points
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static ArrayList<Long> computeWeights(ArrayList<Vector> points, ArrayList<Vector> centers) {
        Long weights[] = new Long[centers.size()];
        Arrays.fill(weights, 0L);
        for (int i = 0; i < points.size(); ++i) {
            double tmp = euclidean(points.get(i), centers.get(0));
            int mycenter = 0;
            for (int j = 1; j < centers.size(); ++j) {
                if (euclidean(points.get(i), centers.get(j)) < tmp) {
                    mycenter = j;
                    tmp = euclidean(points.get(i), centers.get(j));
                }
            }
            // System.out.println("Point = " + points.get(i) + " Center = " + centers.get(mycenter));
            weights[mycenter] += 1L;
        }
        ArrayList<Long> fin_weights = new ArrayList<>(Arrays.asList(weights));
        return fin_weights;
    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method SeqWeightedOutliers: sequential k-center with outliers
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static ArrayList<Vector> SeqWeightedOutliers(ArrayList<Vector> P,
                                                        ArrayList<Long> W,
                                                        int k, int z,
                                                        double alpha) {

        //precompute the distances among the whole point set: fill up only half matrix
        boolean half_matrix = true;
        double[][] distances = precomputeDistances(P, half_matrix);

        //looking for min distance among first k+z+1 points
        double r = Double.MAX_VALUE;
        for (int i = 0; i < (k + z); i++) {
            for (int j = i + 1; j < (k + z + 1); j++) {
                //check indices validity for matrix distances and retrieve correct distance value
                double distance = distances[i][j];
                if (half_matrix && i > j) distance = distances[j][i];
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
                            if (half_matrix && x > y) distance = distances[y][x];
                            //update weight of the ball
                            if (distance <= ball_radius) {
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
                        if (half_matrix && newcenter > y) distance = distances[y][newcenter];
                        //update total weight of Z and Z
                        if (distance <= ball_radius) {
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
    // Method precomuteDistances: compute distances
    // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    public static double[][] precomputeDistances(ArrayList<Vector> pointSet, boolean half_matrix) {
        double[][] distances = new double[pointSet.size()][pointSet.size()];
        //computing distances among the input set points
        for (int i = 0; i < pointSet.size(); i++) {
            distances[i][i] = 0.0;
            for (int j = i + 1; j < pointSet.size(); j++) {
                distances[i][j] = Math.sqrt(Vectors.sqdist(pointSet.get(i), pointSet.get(j)));
                if (!half_matrix) {
                    distances[j][i] = distances[i][j];
                }
            }
        }

        return distances;
    }

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// Method computeObjective: computes objective function
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    public static double computeObjective(JavaRDD<Vector> points, ArrayList<Vector> centers, int z) {

        return points
                //for each point it is computed the distance to the closest center
                .map((elem) -> {
                    double min = Double.MAX_VALUE;
                    for (Vector center : centers) {
                        double distance = Math.sqrt(Vectors.sqdist(elem, center));
                        if (distance < min) {
                            min = distance;
                        }
                    }
                    return min;
                })
                //it retrieves the biggest z+1 distances and put them in order from the biggest
                .top(z+1)
                //it gets the minimum distance among the z+1 max distances
                .get(z);
    }
}
