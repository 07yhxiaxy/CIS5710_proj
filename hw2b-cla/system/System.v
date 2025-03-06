module gp1 (
	a,
	b,
	g,
	p
);
	input wire a;
	input wire b;
	output wire g;
	output wire p;
	assign g = a & b;
	assign p = a | b;
endmodule
module gp4 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [3:0] gin;
	input wire [3:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [2:0] cout;
	assign cout[0] = (pin[0] & cin) | gin[0];
	assign cout[1] = (((pin[0] & pin[1]) & cin) | (gin[0] & pin[1])) | gin[1];
	assign cout[2] = (((((pin[0] & pin[1]) & pin[2]) & cin) | ((gin[0] & pin[1]) & pin[2])) | (pin[2] & gin[1])) | gin[2];
	assign gout = ((gin[3] | (pin[3] & gin[2])) | ((pin[3] & pin[2]) & gin[1])) | (((pin[3] & pin[2]) & pin[1]) & gin[0]);
	assign pout = ((pin[3] & pin[2]) & pin[1]) & pin[0];
endmodule
module gp8 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [7:0] gin;
	input wire [7:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [6:0] cout;
	wire [2:0] cout_tmp_low;
	wire [2:0] cout_tmp_high;
	wire gout_tmp;
	wire pout_tmp;
	wire gout_tmp1;
	wire pout_tmp1;
	wire carry;
	assign carry = gout_tmp | (pout_tmp & cin);
	gp4 gp4_low(
		.gin(gin[3:0]),
		.pin(pin[3:0]),
		.cin(cin),
		.gout(gout_tmp),
		.pout(pout_tmp),
		.cout(cout_tmp_low[2:0])
	);
	gp4 gp4_high(
		.gin(gin[7:4]),
		.pin(pin[7:4]),
		.cin(carry),
		.gout(gout_tmp1),
		.pout(pout_tmp1),
		.cout(cout_tmp_high[2:0])
	);
	assign cout = {cout_tmp_high, carry, cout_tmp_low};
	assign gout = gout_tmp1 | (gout_tmp & pout_tmp1);
	assign pout = pout_tmp & pout_tmp1;
endmodule
module cla (
	a,
	b,
	cin,
	sum
);
	input wire [31:0] a;
	input wire [31:0] b;
	input wire cin;
	output wire [31:0] sum;
	wire [31:0] gtemp;
	wire [31:0] ptemp;
	wire [6:0] cout_temp [3:0];
	wire carry1;
	wire carry2;
	wire carry3;
	wire carry4;
	wire [3:0] gout_temp;
	wire [3:0] pout_temp;
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < 32; _gv_i_1 = _gv_i_1 + 1) begin : gp1_block
			localparam i = _gv_i_1;
			gp1 a0(
				.a(a[i]),
				.b(b[i]),
				.g(gtemp[i]),
				.p(ptemp[i])
			);
		end
	endgenerate
	gp8 a1(
		.gin(gtemp[7:0]),
		.pin(ptemp[7:0]),
		.cin(cin),
		.gout(gout_temp[0]),
		.pout(pout_temp[0]),
		.cout(cout_temp[0])
	);
	gp8 a2(
		.gin(gtemp[15:8]),
		.pin(ptemp[15:8]),
		.cin(carry1),
		.gout(gout_temp[1]),
		.pout(pout_temp[1]),
		.cout(cout_temp[1])
	);
	gp8 a3(
		.gin(gtemp[23:16]),
		.pin(ptemp[23:16]),
		.cin(carry2),
		.gout(gout_temp[2]),
		.pout(pout_temp[2]),
		.cout(cout_temp[2])
	);
	gp8 a4(
		.gin(gtemp[31:24]),
		.pin(ptemp[31:24]),
		.cin(carry3),
		.gout(gout_temp[3]),
		.pout(pout_temp[3]),
		.cout(cout_temp[3])
	);
	assign carry1 = gout_temp[0] | (pout_temp[0] & cin);
	assign carry2 = gout_temp[1] | (pout_temp[1] & carry1);
	assign carry3 = gout_temp[2] | (pout_temp[2] & carry2);
	assign carry4 = gout_temp[3] | (pout_temp[3] & carry3);
	assign sum[0] = (cin ^ a[0]) ^ b[0];
	genvar _gv_j_1;
	generate
		for (_gv_j_1 = 1; _gv_j_1 < 8; _gv_j_1 = _gv_j_1 + 1) begin : sum1_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[0][j - 1] ^ a[j]) ^ b[j];
		end
	endgenerate
	assign sum[8] = (carry1 ^ a[8]) ^ b[8];
	generate
		for (_gv_j_1 = 9; _gv_j_1 < 16; _gv_j_1 = _gv_j_1 + 1) begin : sum2_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[1][j - 9] ^ a[j]) ^ b[j];
		end
	endgenerate
	assign sum[16] = (carry2 ^ a[16]) ^ b[16];
	generate
		for (_gv_j_1 = 17; _gv_j_1 < 24; _gv_j_1 = _gv_j_1 + 1) begin : sum3_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[2][j - 17] ^ a[j]) ^ b[j];
		end
	endgenerate
	assign sum[24] = (carry3 ^ a[24]) ^ b[24];
	generate
		for (_gv_j_1 = 25; _gv_j_1 < 32; _gv_j_1 = _gv_j_1 + 1) begin : sum4_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[3][j - 25] ^ a[j]) ^ b[j];
		end
	endgenerate
endmodule
module SystemDemo (
	btn,
	led
);
	input wire [6:0] btn;
	output wire [7:0] led;
	wire [31:0] sum;
	cla cla_inst(
		.a(32'd26),
		.b({27'b000000000000000000000000000, btn[1], btn[2], btn[5], btn[4], btn[6]}),
		.cin(1'b0),
		.sum(sum)
	);
	assign led = sum[7:0];
endmodule