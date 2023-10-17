import breeze.optimize.linear.LinearProgram;
import org.apache.spark.SparkConf;
import org.apache.spark.api.java.JavaPairRDD;
import org.apache.spark.api.java.JavaRDD;
import org.apache.spark.api.java.JavaSparkContext;
import org.apache.spark.sql.sources.In;
import scala.Tuple2;
import shapeless.Tuple;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class G044HW1 {

    public static void main(String[] args) throws IOException {

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // CHECKING NUMBER OF CMD LINE PARAMETERS
        // Parameters are: num_partitions, popularity_flag, country_filter, <path_to_file>
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        if (args.length != 4) {
            throw new IllegalArgumentException("USAGE: num_partitions popularity_flag country_filter file_path");
        }

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // SPARK SETUP
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        SparkConf conf = new SparkConf(true).setAppName("G044HW1");
        JavaSparkContext sc = new JavaSparkContext(conf);
        sc.setLogLevel("WARN");

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 01. INPUT READING & ROWS PRINTING
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        // SETTING INPUT VARIABLES RECEIVED BY CLI INPUT

        // Input correctness check and variable assignment
        int K, H;
        String S, file_path;
        try {
            K = Integer.parseInt(args[0]);
            H = Integer.parseInt(args[1]);
            S = args[2];
            file_path = args[3];
        } catch (Exception e) {
            throw new IllegalArgumentException("USAGE: num_partitions popularity_flag country_filter file_path");
        }

        // SETTING GLOBAL VARIABLES

        // Read input file and subdivide it into K random partitions
        JavaRDD<String> rawData = sc.textFile(file_path).repartition(K).cache();

        //Compute and show number of rows of the input documents
        long rows;
        rows = rawData.count();
        System.out.println("Number of rows = " + rows);

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 02. productCustomer RDD AND RELATIVE PAIRS NUMBER PRINTING
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        /* ONE ROUND ALGORITHM

        MAP PHASE:  from each row create a ((productID, customerID), qty) pair that respect the requirements
        REDUCE PHASE:   each element is a ((productID, customerID), L_q) pair,
                        where L_q is a list of qty related to (productID, customerID) key
                        for each ((productID, customerID), qty) pair creates a pair (productID, customerID) that implement
                        the final desired RDD

        JavaPairRDD<String, Integer> productCustomer;
        productCustomer = rawData.flatMapToPair((iteration) -> { // <--MAP PHASE (R1)
          String[] tokens = iteration.split(",");
          String productID = tokens[1];
          Integer qty = Integer.parseInt(tokens[3]);
          String country = tokens[7];
          Integer customerID = Integer.parseInt(tokens[6]);
          ArrayList<Tuple2<Tuple2<String, Integer>,Integer>> pairs = new ArrayList<>();
          if(qty>0 && (S.equals("all") || S.equals(country))){
            Tuple2<String, Integer> pair = new Tuple2<>(productID,customerID);
            pairs.add(new Tuple2<>(pair, qty));
          }
          return pairs.iterator();
        })
        .groupByKey()// (pair) -> new Random().nextInt(K)))
        .flatMapToPair((element) -> { // <-- REDUCE PHASE (R1)
          ArrayList<Tuple2<String, Integer>> pairs = new ArrayList<>();
          pairs.add(element._1());
          return pairs.iterator();
        });
        long productCostumerPairs = productCustomer.count();
        System.out.println("Product-Customer Pairs = " + productCostumerPairs);

        END OF ONE ROUND ALGORITHM
        */

            // TWO ROUND ALGORITHM

        /*
        ROUND 1
            MAP PHASE:  from each row of the input dataset, the info of interest are retrieved
                        and it's created a (random_key, (productID, customerID)) pair according to the CLI requests,
                        where random_key is a random integer from 0 to K-1 and it detects a partition

            REDUCE PHASE:  each element is a (random_key, L_pc) pair, where L_pc is a list of pairs (productID, customerID)
                           for each unique element of L_pc, it's created a ((productID, customerID), 1) pair

        ROUND 2
            MAP PHASE:  EMPTY

            REDUCE PHASE:   each element is a ((productID, customerID), L) pair, where L is a list of one or more 1's
                            (the sum of the overall elements of L represents how many (productID, customerID) pairs are presented in the dataset)
                            for each pair, the (productID, customerID) key is retrieved to create the new RDD
         */

        JavaPairRDD<String, Integer> productCustomer;
        productCustomer = rawData
                .flatMapToPair((iteration) -> { // <--MAP PHASE (R1)
                    //retrieval info of interest from the row
                    String[] tokens = iteration.split(",");
                    String productID = tokens[1];
                    Integer qty = Integer.parseInt(tokens[3]);
                    String country = tokens[7];
                    Integer customerID = Integer.parseInt(tokens[6]);
                    //ArrayList is used in order to pass an iterator
                    ArrayList<Tuple2<String, Integer>> pairs = new ArrayList<>();
                    //check if the entry respect requirements
                    if (qty > 0 && (S.equals("all") || S.equals(country))) {
                        pairs.add(new Tuple2<>(productID, customerID));
                    }
                    return pairs.iterator();
                })
                .groupBy((pair) -> new Random().nextInt(K)) // <-- PARTITIONING AND GROUPING BY RANDOM KEY FROM 0 TO K-1
                .flatMapToPair((element) -> { // <-- REDUCE PHASE (R1)
                    ArrayList<Tuple2<Tuple2<String, Integer>, Integer>> pairs = new ArrayList<>();
                    //HashMap used to check the presence of duplicates in the partition
                    HashMap<Tuple2<String, Integer>, Integer> singlePairs = new HashMap<>();
                    for (Tuple2<String, Integer> pair : element._2()) {
                        //check if a pair (productID, customerID) has been already inserted
                        if (!singlePairs.containsKey(pair)) {
                            singlePairs.put(pair, 1);
                            pairs.add(new Tuple2<>(pair, 1));
                        }
                    }
                    return pairs.iterator();
                })
                // <-- EMPTY MAP PHASE (R2)
                .groupByKey() // <-- GROUPING BY KEY
                .flatMapToPair((element) -> { // <-- REDUCE PHASE (R2)
                    ArrayList<Tuple2<String, Integer>> pairs = new ArrayList<>();
                    pairs.add(element._1());
                    return pairs.iterator();
                });
        long productCostumerPairs = productCustomer.count();
        System.out.println("Product-Customer Pairs = " + productCostumerPairs);

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 03. POPULARITY OF EACH PRODUCT BY USING mapPartitionsToPair
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        /*
        ROUND 1
            MAP PHASE:  EMPTY (RDD of the previous point is used)

            REDUCE PHASE:   each element is a (i, L_pc) pair, where i denotes the i-partition and
                            L_pc is a list of (productID, customerID)
                            for each partition i, it creates a pair (productID, count_customerID) where count_customerID is an integer
                            that represents the number of customer related to the same productID (the partial_total)

        ROUND 2
            MAP PHASE:  EMPTY

            REDUCE PHASE:   each element is a (productID, L) pair, where L contains one element for each partition (K at most)
                            and each element is the total number of customerID with productID as key in that partition
                            for each pair, sum together all the K (at most) values of L to obtain the pair (productID, popularity)
                            and produce the new RDD
         */

        JavaPairRDD<String, Integer> productPopularity1;
        productPopularity1 = productCustomer
                // <-- EMPTY MAP PHASE (R1)
                .mapPartitionsToPair((iterator) -> { // <-- REDUCE PHASE (R1)
                    //HashMap that uses as key the productID and as value the number of customerID with the same productID
                    HashMap<String, Integer> counts = new HashMap<>();
                    //compute the partial_total for each key
                    while(iterator.hasNext()){
                        Tuple2<String, Integer> pair = iterator.next();
                        counts.put(pair._1(), 1 + counts.getOrDefault(pair._1(), 0));
                    }
                    //creation of the iterator for the successive phase
                    ArrayList<Tuple2<String, Integer>> pairs = new ArrayList<>();
                    for(Map.Entry<String, Integer> pair : counts.entrySet()){
                        pairs.add(new Tuple2<>(pair.getKey(), pair.getValue()));
                    }
                    return pairs.iterator();
                })
                // <-- EMPTY MAP PHASE (R2)
                .groupByKey() // <-- GROUPING BY productID KEY (all partitions are exploited)
                .mapValues((values) -> { // <-- REDUCE PHASE (R2)
                    //compute the total for each productID
                    int sum = 0;
                    for(int value : values) sum += value;
                    return sum;
                });

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 04. POPULARITY OF EACH PRODUCT WITHOUT USING mapPartitionsToPair
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        /*
        ROUND 1
            MAP PHASE:  each (productID, customerID) pair is mapped into a ((random_partition, productID), 1),
                        where random_partition is a random integer from 0 to K-1 that divide the data into K partition
                        (in order to reduce the local space)

            REDUCE PHASE:   each element is a ((i, productID), L_c) pair, where i denotes the i-partition and
                            L_c is a list of one or more 1's (as many customerID with the same i and productID)
                            for each partition i, it creates a pair ((i, productID), count_customerID) where count_customerID is an integer
                            that represents the number of customer related to the same productID and same partition i
                            (this number represents the partial_total)

        ROUND 2
            MAP PHASE:  each element ((i, productID), count_customerID) is mapped into (productID, count_customerID)

            REDUCE PHASE:   each element is a (productID, L) pair, where L contains all the partial_total for each productID
                            computed in the previous round (at most K)
                            for each pair, sum together all the values of L to obtain the pair (productID, popularity)
                            and produce the new RDD
         */

        JavaPairRDD<String, Integer> productPopularity2;
        productPopularity2 = productCustomer
                .mapToPair((pair) ->{ // <-- MAP PHASE (R1)
                    //creating pairs ((partition, productID), 1)
                    Tuple2<Integer, String> key = new Tuple2<>(new Random().nextInt(K), pair._1());
                    Tuple2<Tuple2<Integer, String>, Integer> new_pair = new Tuple2<>(key, 1);
                    return new_pair;
                })
                .reduceByKey((v1, v2) -> v1 + v2) // <-- REDUCE PHASE (R1)
                .mapToPair((pair) -> { // <-- MAP PHASE (R2)
                    //creating pairs (productID, partial_total)
                    String key = pair._1()._2();
                    Tuple2<String, Integer> new_pair = new Tuple2<>(key, pair._2());
                    return new_pair;
                })
                //compute the overall total (popularity) for each productID
                .reduceByKey((v1, v2) -> v1 + v2); // <-- REDUCE PHASE (R2)

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 05. (H>0) SHOW THE H HIGHEST POPULARITY PRODUCTS FROM productPopularity1
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        if(H>0){
            //setting as key the popularity and as value the productID to sort them by popularity by using sortByKey() method
            List<Tuple2<Integer, String>> popularityList;
            popularityList = productPopularity1
                    .flatMapToPair((pair) -> {
                        ArrayList<Tuple2<Integer, String>> temp = new ArrayList<>();
                        Tuple2<Integer, String> new_pair = new Tuple2<>(pair._2(), pair._1());
                        temp.add(new_pair);
                        return temp.iterator();
                    })
                    .sortByKey(false).take(H);
            //printing top H popular products
            System.out.println("Top " + H + " Products and their Popularities");
            for(int i=0; i<popularityList.size(); i++) {
                Tuple2<Integer, String> pair = popularityList.get(i);
                String productID = pair._2();
                Integer popularity = pair._1();
                System.out.print("Product " + productID + " Popularity " + popularity + "; ");
            }
        }

        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 07. (H=0) SHOW productPopularity1 AND productPopularity2 IN INCREASING ORDER BY productID
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        if(H==0){
            List<Tuple2<String, Integer>> list;

            //sorting by productID by using sorByKey() method and then printing the ordered RDD

            //productPopularity1
            list = productPopularity1.sortByKey().collect();
            System.out.println("productPopularity1:");
            for(int i=0; i<list.size(); i++) {
                Tuple2<String, Integer> pair = list.get(i);
                System.out.print("Product: " + pair._1() + " Popularity: " + pair._2() + "; ");
            }

            //productPopularity2
            list = productPopularity2.sortByKey().collect();
            System.out.println("\nproductPopularity2:");
            for(int i=0; i<list.size(); i++) {
                Tuple2<String, Integer> pair = list.get(i);
                System.out.print("Product: " + pair._1() + " Popularity: " + pair._2() + "; ");
            }

        }
        System.out.println("");
    }

}
