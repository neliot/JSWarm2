/** MyPRNG.java
  * A hand-rolled pseudo-random number generator following the java recipe
  * (see wiki on linear-congruential PRNGs and the documentaion for 
  * java.util.Random)
  *
  * The linear congruence is x <- (a*x + c) % m where
  *   the modulus m = 2^48, a = 0x5DEECE66D, c = 11 = 0xB;
  *   x is initialised to a seed value and repeatedly updated by fcn next().
  *
  * The java implementation embellishes this scheme by
  *   (1) preprosessing the seed: it is XORed with a then reduced mod m 
  *   (2) next() takes a parameter numBits 1..32, performs the congruential
  *   update, then outputs numBits-bit value by discarding the bottom
  *   (48 - numBits) bits. 
  *   Eg nextInt() calls next(32) & outputs a 32-bit value by discarding 16 bits.
         nextLong() outputs a 64-bit integer built from two 32-bit integers.
  *
  * This code offers System.nanoTime() as a default seed if none is given to
  * the constuctor. This is the outputof a very fast-ticking clock in the JVM 
  * (a time in nanoseconds, not necessarily with ns resolution!). The built-in
  * java.util.Random implementation has a default seed too, but the doc doesn't
  * specify it in detail.
  */ 
class PRNG {
  private int  bitSz = 48;          //Ref: wiki on Linear PRNG,
  private long m = 1L << bitSz,     //     doc for java.util.Random class
               a = 0x5DEECE66DL,
               c = 11L,
               x;
  
  /** Construct a PRNG with an optionally specified seed, or if not specified,
   *  an output of System.nanoTime() 
   */
  public PRNG(long s) { reseed(s); }
  public PRNG()       { reseed(System.nanoTime()); }
  
  /** (Re)initialise seed, preprocessing as per java documentation */
  void reseed(long s) {
    x = (s ^ a) & (m-1);
  }
  
  /** Core generator method
   * numBits is 'size' of output, 1--32 in order fit into an int
   * (cf java documentation for util.Random) */
  private int next(int numBits) {
    x = (x * a + c) & (m - 1);
    return (int)(x >>> (bitSz - numBits));
  }
  
  /** output methods */
  public int nextInt() {
    return next(32); //= seed >>> 16 per wiki article
  }

  public int nextInt(int bound) {
   if (bound <= 0)
     throw new IllegalArgumentException("bound must be positive");

   if ((bound & -bound) == bound)  // i.e., bound is a power of 2
     return (int)((bound * (long)next(31)) >> 31);

   int bits, val;
   do {
       bits = next(31);
       val = bits % bound;
   } while (bits - val + (bound-1) < 0);
   return val;
 }

  /** A long is built of two 32-bit integers */
  public long nextLong() {
    long outlo = (long)nextInt(), 
         outhi = (long)nextInt();
    return (outhi << 32) + outlo; //cf util.Random java documentation
  }

  public float nextFloat() {
    return next(24) / ((float)(1 << 24));
 }  
 
  public double nextDouble() {
    return (((long)next(26) << 27) + next(27)) / (double)(1L << 53);
 }
}
 
 
//   /***************** Testing, playing,  etc ***********************/
  
//   /** Generate ints in tandem with a java.util.Random instance;
//    *  Output them along with the difference (should be 0)   */
//   public static void compareIntsWithJava(long seed, int n) {
//     MyPRNG gen = new MyPRNG(seed);
//     java.util.Random genj = new java.util.Random(seed);
//     int r;
//     for (int k = 0; k < n; k++) {
//       r = gen.nextInt();  
//       System.out.printf("%08x; %x   ", r, r - genj.nextInt());
//       if (k%8 == 7) System.out.println();
//     }
//     System.out.println();
//   }
  
//   /** Generate & output floats in tandem with a java.util.Random instance */
//   public static void compareFloatsWithJava(long seed, int n) {
//     MyPRNG gen = new MyPRNG(seed);
//     java.util.Random genj = new java.util.Random(seed);
//     for (int k = 0; k < n; k++) {
//       System.out.printf("%f; %f    ", gen.nextFloat(), genj.nextFloat());
//       if (k%5 == 4) System.out.println();
//     }
//     System.out.println();
//   }
  
//   /** Generate & output floats in tandem with a java.util.Random instance */
//   public static void compareDoublesWithJava(long seed, int n) {
//     MyPRNG gen = new MyPRNG(seed);
//     java.util.Random genj = new java.util.Random(seed);
//     for (int k = 0; k < n; k++) {
//       System.out.printf("%.10g; %.10g    ", gen.nextDouble(), genj.nextDouble());
//       if (k%3 == 2) System.out.println();
//     }
//     System.out.println();
//   }
  
//   public static void main(String[] args) {
//     if (args.length < 3) {
//       System.out.println("Usage: java MyPRNG I|F|D <num> <seed>");
//       return;
//     }
//     long seed = Long.parseLong(args[2]);
//     int  num = Integer.parseInt(args[1]);
//     char opt = Character.toUpperCase(args[0].charAt(0));
//     if (opt == 'I')
//       compareIntsWithJava(seed, num);
//     else if (opt == 'F')
//       compareFloatsWithJava(seed, num);
//     else if (opt == 'D')
//       compareDoublesWithJava(seed, num);
//   }
  
// }
