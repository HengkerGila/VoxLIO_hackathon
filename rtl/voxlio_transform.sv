// Stage 1: p = R q + t. Three pipeline stages: products, sums, quantisation.
// The pose must stay constant while points are in flight.

module voxlio_transform
  import voxlio_pkg::*;
(
  input  logic                      clk,
  input  logic                      rst_n,
  input  logic                      valid_in,
  input  logic signed [COORD_W-1:0] qx,
  input  logic signed [COORD_W-1:0] qy,
  input  logic signed [COORD_W-1:0] qz,
  input  logic [9*NORMAL_W-1:0]     pose_r,   // row-major, element i at [i*NORMAL_W +: NORMAL_W]
  input  logic [3*COORD_W-1:0]      pose_t,   // element i at [i*COORD_W +: COORD_W]
  output logic                      valid_out,
  output logic signed [COORD_W-1:0] px,
  output logic signed [COORD_W-1:0] py,
  output logic signed [COORD_W-1:0] pz
);
  localparam int PROD_W = NORMAL_W + COORD_W;  // exact product, COORD_F + NORMAL_F fraction bits
  localparam int SUM_W  = PROD_W + 3;          // three products plus t
  localparam int DROP   = NORMAL_F;            // back to COORD_F fraction bits

  logic signed [NORMAL_W-1:0] r [9];
  logic signed [COORD_W-1:0]  t [3];
  logic signed [COORD_W-1:0]  q [3];

  always_comb begin
    for (int i = 0; i < 9; i++) r[i] = pose_r[i*NORMAL_W +: NORMAL_W];
    for (int i = 0; i < 3; i++) t[i] = pose_t[i*COORD_W +: COORD_W];
    q[0] = qx;
    q[1] = qy;
    q[2] = qz;
  end

  logic signed [PROD_W-1:0] prod [9];
  logic signed [SUM_W-1:0]  sum  [3];
  logic signed [COORD_W-1:0] rounded [3];
  logic v1, v2;

  always_ff @(posedge clk) begin
    for (int a = 0; a < 3; a++) begin
      for (int i = 0; i < 3; i++) begin
        prod[3*a+i] <= r[3*a+i] * q[i];
      end
      sum[a] <= SUM_W'(prod[3*a]) + SUM_W'(prod[3*a+1]) + SUM_W'(prod[3*a+2])
                + (SUM_W'(t[a]) <<< DROP);
    end
    px <= rounded[0];
    py <= rounded[1];
    pz <= rounded[2];
  end

  generate
    for (genvar a = 0; a < 3; a++) begin : g_round
      voxlio_round_sat #(.IN_W(SUM_W), .DROP(DROP), .OUT_W(COORD_W), .ROUND(ROUND_TO_NEAREST))
        u_round (.in(sum[a]), .out(rounded[a]));
    end
  endgenerate

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1 <= 1'b0;
      v2 <= 1'b0;
      valid_out <= 1'b0;
    end else begin
      v1 <= valid_in;
      v2 <= v1;
      valid_out <= v2;
    end
  end
endmodule
