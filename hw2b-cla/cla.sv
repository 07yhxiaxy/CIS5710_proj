`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

module clc(input wire a, b, cin,
           output wire g, p, s);
   gp1 inst1(.a(a), .b(b), .g(g), .p(p));
   assign s = cin ^ a ^ b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

   // TODO: your code here
   assign cout[0] = (pin[0] & cin) | gin[0];
   assign cout[1] = (pin[0] & pin[1] & cin) | (gin[0] & pin[1]) | gin[1];
   assign cout[2] = (pin[0] & pin[1] & pin[2] & cin) | (gin[0] & pin[1] & pin[2]) | (pin[2] & gin[1]) | gin[2];
   assign gout = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]);
   assign pout = pin[3] & pin[2] & pin[1] & pin[0];

endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   // TODO: your code here
   logic [2:0]cout_tmp_low;
   logic [2:0]cout_tmp_high;
   logic gout_tmp, pout_tmp, gout_tmp1, pout_tmp1, carry;
   assign carry = gout_tmp | (pout_tmp & cin);
   gp4 gp4_low(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin), .gout(gout_tmp), .pout(pout_tmp), .cout(cout_tmp_low[2:0]));
   gp4 gp4_high(.gin(gin[7:4]), .pin(pin[7:4]), .cin(carry), .gout(gout_tmp1), .pout(pout_tmp1), .cout(cout_tmp_high[2:0]));
   assign cout = {cout_tmp_high, carry, cout_tmp_low};
   assign gout = gout_tmp1 | (gout_tmp & pout_tmp1);
   assign pout = pout_tmp & pout_tmp1;

endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);
   // TODO: your code here

   // Temporary generate and propagate signals
   logic[31:0] gtemp, ptemp;
   logic[6:0] cout_temp[3:0];  // 7-bit carry-out for each gp8 block
   logic carry1, carry2, carry3, carry4;
   logic[3:0] gout_temp, pout_temp;

   // Generate-propagate computation for each bit
   genvar i;
   generate
      for (i = 0; i < 32; i = i + 1) begin : gp1_block
         gp1 a0 (.a(a[i]), .b(b[i]), .g(gtemp[i]), .p(ptemp[i]));
      end
   endgenerate

   // 8-bit blocks for carry-lookahead
   gp8 a1 (.gin(gtemp[7:0]), .pin(ptemp[7:0]), .cin(cin), .gout(gout_temp[0]), .pout(pout_temp[0]), .cout(cout_temp[0]));
   gp8 a2 (.gin(gtemp[15:8]), .pin(ptemp[15:8]), .cin(carry1), .gout(gout_temp[1]), .pout(pout_temp[1]), .cout(cout_temp[1]));
   gp8 a3 (.gin(gtemp[23:16]), .pin(ptemp[23:16]), .cin(carry2), .gout(gout_temp[2]), .pout(pout_temp[2]), .cout(cout_temp[2]));
   gp8 a4 (.gin(gtemp[31:24]), .pin(ptemp[31:24]), .cin(carry3), .gout(gout_temp[3]), .pout(pout_temp[3]), .cout(cout_temp[3]));

   // Carry propagation
   assign carry1 = gout_temp[0] | (pout_temp[0] & cin);
   assign carry2 = gout_temp[1] | (pout_temp[1] & carry1);
   assign carry3 = gout_temp[2] | (pout_temp[2] & carry2);
   assign carry4 = gout_temp[3] | (pout_temp[3] & carry3);

   // Sum computation
   assign sum[0] = cin ^ a[0] ^ b[0];

   genvar j;
   generate
      for (j = 1; j < 8; j = j + 1) begin : sum1_block
         assign sum[j] = cout_temp[0][j - 1] ^ a[j] ^ b[j];
      end
   endgenerate

   assign sum[8] = carry1 ^ a[8] ^ b[8];

   generate
      for (j = 9; j < 16; j = j + 1) begin : sum2_block
         assign sum[j] = cout_temp[1][j - 9] ^ a[j] ^ b[j];
      end
   endgenerate

   assign sum[16] = carry2 ^ a[16] ^ b[16];

   generate
      for (j = 17; j < 24; j = j + 1) begin : sum3_block
         assign sum[j] = cout_temp[2][j - 17] ^ a[j] ^ b[j];
      end
   endgenerate

   assign sum[24] = carry3 ^ a[24] ^ b[24];

   generate
      for (j = 25; j < 32; j = j + 1) begin : sum4_block
         assign sum[j] = cout_temp[3][j - 25] ^ a[j] ^ b[j];
      end
   endgenerate

endmodule
