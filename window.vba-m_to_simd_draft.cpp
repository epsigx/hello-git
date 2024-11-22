// =================================================================================== //
//                           (bg mask)        ||                           (sp_mask)   //
// =================================================================================== //
// d24:0                                      ||                                       //
// d25:0                                      || obj-blend mask                        //
// d26:disable mask (2ndTarget, fixed)        || disable mask (2ndTarget, fixed)       //
// d27:disable mask (1stTarget)               || disable mask (1stTarget)              //
// d28:1                                      || 0                                     //
// d29:pri                                    || pri                                   //
// d30:pri                                    || pri                                   //
// d31:backdrop mask                          || backdrop mask                         //
// =================================================================================== //
// min_c0c1         = pmin c0, c1                                               // min (0, 1)
// min_c2c3         = pmin c2, c3                                               // min (2, 3)
// min_c0c1c2c3     = pmin min_c0c1, min_c2c3                                   // min (0, 1, 2, 3)
// min_c0c1c2c3sp   = pmin sp, min_c0c1c2c3                                     // min (0, 1, 2, 3, sp)
// not_min_c0c1c2c3 = pmax min_c0c1, min_c2c3                                   // not min
// max_c0c1         = pmax c0, c1                                               // 
// max_c2c3         = pmax c2, c3                                               // 
// not_max_c0c1c2c3 = pmin max_c0c1, max_c2c3                                   // not max
// under_sprite     = pmin not_min_c0c1c2c3, not_max_c0c1c2c3 (obj-alpha)       // get the pixel at the under of the sprit (for obj-alpha render)
// ============================================================================ //
// unsigned int min_c0c1 = std::min (c[0], c[1]);
// unsigned int max_c0c1 = std::max (c[0], c[1]);
// unsigned int min_c3c4 = std::min (c[3], c[4]);
// unsigned int max_c3c4 = std::max (c[3], c[4]);
// unsigned int s0 = std::min (min_c0c1, min_c3c4); // [0]
// unsigned int s1 = std::max (min_c0c1, min_c3c4); // [1]
// unsigned int s3 = std::min (max_c0c1, max_c3c4); // [3] 
// unsigned int s4 = std::max (max_c0c1, max_c3c4); // [4]
// unsigned int Tmin = std::min (c[2], s0); // min
// unsigned int mid_ = std::max (c[2], s0);
// unsigned int Tmax = std::max (s4, mid_); // max 
// unsigned int mid  = std::min (s4, mid_);
// unsigned int smin_= std::min (mid, s1);
// unsigned int smin = std::min (smin_, s3);//  second min
// ============================================================================

//                    diseff1 (trans...)
//                       1
//                       1
//                       1
//                       1


//         aaaa bbbb cccc dddd 



///
//       nn 0 commnad 1

//  d7 | d6 d5 | d4 d3 | d2              
         mp mp   sp sp

// mp mp sp sp sp obm 1st 2nd



//     000
//     

//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    e                       d                           c   b                           a

// shlpd a, 21 
// shlpd b, 14 
// shlpw c, 13 
// shlpw d, 6
// srapd e, 31

//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    e                       d               d'          c   b                           a           a'

// shlpd a, 21 
// shlpd b, 14 
// shlpw c, 13 
// shlpw d, 6
// srapd e, 31







//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    -   4   4   4   4   4   -   +   3   3   3   3   3   +   -   2   2   2   2   2   -   +  1  1  1  1  +  -  0  0  0  - 
//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    -   4   4   4   4   4   -   +   3   3   3   3   3   +   -   2   2   2   2   2   -   +  1  1  1  1  +  -  0  0  0  -        
//                                                                                                    
//                                                                                                    
//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    -   4   4   4   -   0   +   3   3   3   +   0   3   +   -   2   2   2   2   2   -   +  1  1  1  1  +  -  0  0  0  - 
//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    -   4   4   4   4   4   -   +   3   3   3   3   3   +   -   2   2   2   2   2   -   +  1  1  1  1  +  -  0  0  0  -   



// =================================================================================== //
//                           (bg mask)        ||                           (sp_mask)   //
// =================================================================================== //
// d24:0                                      ||                                       //
// d25:0                                      ||                                       //
// d26:0                                      || obj-blend mask                        //
// d27:disable mask (2ndTarget, fixed)        || disable mask (2ndTarget, fixed)       //
// d28:disable mask (1stTarget)               || disable mask (1stTarget)              //
// d29:pri                                    || pri                                   //
// d30:pri                                    || pri                                   //
// d31:pri                                    || pri                                   //
// =================================================================================== //


//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17     |     d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//                                                b           b'                                             a        a'
 
// =====
// 000 
// 001
// 010
// 011
// 100
// 101
// 110 
// 111 


//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    e                       d                           c   b                           a

// shlpd a, 21 
// shlpd b, 14 
// shlpw c, 13 
// shlpw d, 6
// srapd e, 31


//  d31 d30 d29 d28 d27 d26 d25 d24 d23 d22 d21 d20 d19 d18 d17 d16 d15 d14 d13 d12 d11 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0 
//    e                       d                           c   b                           a        a'

// shlpd a, 21 
// shlpd b, 14 
// shlpw c, 13 
// shlpw d, 6
// srapd e, 31