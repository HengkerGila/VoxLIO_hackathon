// Stage 9: J = [p x n, n] for the selected plane. Three pipeline stages:
// products, differences, quantisation to compute_t.

module voxlio_jacobian
  import voxlio_pkg::*;
(
  input  logic                       clk,
  input  logic                       rst_n,
  input  logic                       valid_in,
  input  logic signed [COORD_W-1:0]  px,
  input  logic signed [COORD_W-1:0]  py,
  input  logic signed [COORD_W-1:0]  pz,
  input  logic signed [NORMAL_W-1:0] nx,
  input  logic signed [NORMAL_W-1:0] ny,
  input  logic signed [NORMAL_W-1:0] nz,
  output logic                       valid_out,
  output logic [6*COMPUTE_W-1:0]     j_flat      // J[i] at [i*COMPUTE_W +: COMPUTE_W]
);
  localparam int PROD_W = COORD_W + NORMAL_W;
  localparam int DIFF_W = PROD_W + 1;
  localparam int DROP   = COORD_F + NORMAL_F - COMPUTE_F;
  localparam int SHIFT  = COMPUTE_F - NORMAL_F;     // normals are exact in compute_t

  logic signed [PROD_W-1:0] prod [6];
  logic signed [DIFF_W-1:0] diff [3];
  logic signed [COMPUTE_W-1:0] rounded [3];
  logic signed [NORMAL_W-1:0] n1 [3], n2 [3];
  logic v1, v2;

  always_ff @(posedge clk) begin
    prod[0] <= py * nz;
    prod[1] <= pz * ny;
    prod[2] <= pz * nx;
    prod[3] <= px * nz;
    prod[4] <= px * ny;
    prod[5] <= py * nx;
    n1[0] <= nx;
    n1[1] <= ny;
    n1[2] <= nz;
    for (int i = 0; i < 3; i++) begin
      diff[i] <= DIFF_W'(prod[2*i]) - DIFF_W'(prod[2*i+1]);
      n2[i]   <= n1[i];
    end
    for (int i = 0; i < 3; i++) begin
      j_flat[i*COMPUTE_W +: COMPUTE_W]       <= rounded[i];
      j_flat[(3+i)*COMPUTE_W +: COMPUTE_W]   <= COMPUTE_W'(n2[i]) <<< SHIFT;
    end
  end

  generate
    for (genvar i = 0; i < 3; i++) begin : g_round
      voxlio_round_sat #(.IN_W(DIFF_W), .DROP(DROP), .OUT_W(COMPUTE_W), .ROUND(ROUND_TO_NEAREST))
        u_round (.in(diff[i]), .out(rounded[i]));
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
