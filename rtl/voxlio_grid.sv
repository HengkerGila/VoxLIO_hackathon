// Stages 2 and 3: f = (p - m) * (1/s) per axis, bounds check on f, voxel index.
// Two pipeline stages: constant multiply, then truncate/saturate and compare.

module voxlio_grid
  import voxlio_pkg::*;
(
  input  logic                      clk,
  input  logic                      rst_n,
  input  logic                      valid_in,
  input  logic signed [COORD_W-1:0] px,
  input  logic signed [COORD_W-1:0] py,
  input  logic signed [COORD_W-1:0] pz,
  output logic                      valid_out,
  output logic                      in_grid,
  output logic [IX_W-1:0]           ix,
  output logic [IY_W-1:0]           iy,
  output logic [IZ_W-1:0]           iz
);
  localparam int DIFF_W = COORD_W + 1;
  localparam int PROD_W = DIFF_W + COORD_W;   // 2 * COORD_F fraction bits
  localparam int DROP   = COORD_F;            // down to GRID_F = COORD_F

  // Per-axis constants (plain arrays: unpacked localparam arrays are not
  // portable across tools).
  logic signed [COORD_W-1:0] min_raw   [3];
  logic signed [GRID_W-1:0]  limit_raw [3];
  logic signed [COORD_W-1:0] p         [3];
  always_comb begin
    min_raw[0]   = MAP_X_MIN_RAW;
    min_raw[1]   = MAP_Y_MIN_RAW;
    min_raw[2]   = MAP_Z_MIN_RAW;
    limit_raw[0] = GRID_W'(NX) <<< GRID_F;
    limit_raw[1] = GRID_W'(NY) <<< GRID_F;
    limit_raw[2] = GRID_W'(NZ) <<< GRID_F;
    p[0] = px;
    p[1] = py;
    p[2] = pz;
  end

  logic signed [DIFF_W-1:0] diff [3];
  logic signed [PROD_W-1:0] prod [3];
  logic signed [GRID_W-1:0] f    [3];
  logic        in_axis [3];
  logic v1;

  always_comb begin
    for (int a = 0; a < 3; a++) diff[a] = DIFF_W'(p[a]) - DIFF_W'(min_raw[a]);
  end

  always_ff @(posedge clk) begin
    for (int a = 0; a < 3; a++) prod[a] <= diff[a] * INV_VOXEL_RAW;
  end

  generate
    for (genvar a = 0; a < 3; a++) begin : g_axis
      voxlio_round_sat #(.IN_W(PROD_W), .DROP(DROP), .OUT_W(GRID_W), .ROUND(0))
        u_trunc (.in(prod[a]), .out(f[a]));
    end
  endgenerate

  always_comb begin
    for (int a = 0; a < 3; a++) in_axis[a] = (f[a] >= 0) && (f[a] < limit_raw[a]);
  end

  always_ff @(posedge clk) begin
    in_grid <= in_axis[0] && in_axis[1] && in_axis[2];
    ix <= f[0][GRID_F +: IX_W];
    iy <= f[1][GRID_F +: IY_W];
    iz <= f[2][GRID_F +: IZ_W];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1 <= 1'b0;
      valid_out <= 1'b0;
    end else begin
      v1 <= valid_in;
      valid_out <= v1;
    end
  end
endmodule
