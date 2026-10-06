// Stage 11: H += J^T J (packed upper triangle), g += J^T r, cost += r^2,
// inlier_count += 1. Two pipeline stages: products, then the 64-bit adds.
//
// The products are exact, as in the C++ model, but formed from the low
// JAC_MUL_W bits of each Jacobian entry and the low RES_MUL_W bits of the
// residual: every J entry is bounded by twice the grid's corner radius and
// every accumulated residual by the threshold, so the slices lose nothing.
// Simulation asserts that.

module voxlio_accumulate
  import voxlio_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,
  input  logic                        clear,
  input  logic                        valid,
  input  logic [6*COMPUTE_W-1:0]      j_flat,
  input  logic signed [COMPUTE_W-1:0] r,
  output logic [21*ACCUM_W-1:0]       h_flat,     // H[k] at [k*ACCUM_W +: ACCUM_W]
  output logic [6*ACCUM_W-1:0]        g_flat,
  output logic [ACCUM_W-1:0]          cost,
  output logic [31:0]                 inlier_count
);
  localparam int JJ_W = 2 * JAC_MUL_W;
  localparam int JR_W = JAC_MUL_W + RES_MUL_W;
  localparam int RR_W = 2 * RES_MUL_W;

  logic signed [JAC_MUL_W-1:0] j [6];
  logic signed [RES_MUL_W-1:0] rs;
  always_comb begin
    for (int i = 0; i < 6; i++) j[i] = j_flat[i*COMPUTE_W +: JAC_MUL_W];
    rs = r[RES_MUL_W-1:0];
  end

`ifdef VERILATOR
  // The dropped high bits must all equal the sign bit of the kept slice.
  logic signed [COMPUTE_W-1:0] j_full [6];
  always_comb for (int i = 0; i < 6; i++) j_full[i] = j_flat[i*COMPUTE_W +: COMPUTE_W];

  always_ff @(posedge clk) begin
    if (valid) begin
      for (int i = 0; i < 6; i++) begin
        if (j_full[i][COMPUTE_W-1:JAC_MUL_W-1] != {(COMPUTE_W - JAC_MUL_W + 1){j_full[i][JAC_MUL_W-1]}})
          $error("Jacobian entry %0d does not fit JAC_MUL_W bits", i);
      end
      if (r[COMPUTE_W-1:RES_MUL_W-1] != {(COMPUTE_W - RES_MUL_W + 1){r[RES_MUL_W-1]}})
        $error("residual does not fit RES_MUL_W bits");
    end
  end
`endif

  // Stage 1: products.
  logic signed [JJ_W-1:0] pjj [21];
  logic signed [JR_W-1:0] pjr [6];
  logic signed [RR_W-1:0] prr;
  logic v1;

  // Constant loop bounds; the upper-triangle condition is resolved when the
  // loops are unrolled. k = row + col * (col + 1) / 2.
  always_ff @(posedge clk) begin
    for (int col = 0; col < 6; col++) begin
      for (int row = 0; row < 6; row++) begin
        if (row <= col) pjj[row + col * (col + 1) / 2] <= j[row] * j[col];
      end
      pjr[col] <= j[col] * rs;
    end
    prr <= rs * rs;
  end

  // Stage 2: accumulate.
  logic [ACCUM_W-1:0] h [21];
  logic [ACCUM_W-1:0] g [6];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1 <= 1'b0;
      inlier_count <= '0;
      cost <= '0;
      for (int k = 0; k < 21; k++) h[k] <= '0;
      for (int i = 0; i < 6; i++) g[i] <= '0;
    end else begin
      v1 <= valid;
      if (clear) begin
        v1 <= 1'b0;
        inlier_count <= '0;
        cost <= '0;
        for (int k = 0; k < 21; k++) h[k] <= '0;
        for (int i = 0; i < 6; i++) g[i] <= '0;
      end else if (v1) begin
        inlier_count <= inlier_count + 32'd1;
        cost <= cost + ACCUM_W'(prr);
        for (int k = 0; k < 21; k++) h[k] <= h[k] + ACCUM_W'(pjj[k]);
        for (int i = 0; i < 6; i++) g[i] <= g[i] + ACCUM_W'(pjr[i]);
      end
    end
  end

  always_comb begin
    for (int k = 0; k < 21; k++) h_flat[k*ACCUM_W +: ACCUM_W] = h[k];
    for (int i = 0; i < 6; i++) g_flat[i*ACCUM_W +: ACCUM_W] = g[i];
  end
endmodule
