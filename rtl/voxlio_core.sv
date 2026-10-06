// VoxLIO v0.1 core: one scan point at a time through the eleven stages, with the 27-candidate search walking one candidate per cycle.
// Bit-exact with the fixed-point C++ model (hls/ built with
// -DVOXLIO_FIXED_POINT); testbench/tb_rtl.cpp checks that.
//
// Host protocol: write the scan and the voxel map through the two write
// ports, set num_points and the pose, pulse start, wait for done, read the
// results. Inputs must stay stable while busy. The map is never written by
// the core.

module voxlio_core
  import voxlio_pkg::*;
(
  input  logic                   clk,
  input  logic                   rst_n,

  // Scan buffer write port: word i = {z, y, x} of point i, coord_t each.
  input  logic                   scan_we,
  input  logic [SCAN_AW-1:0]     scan_waddr,
  input  logic [SCAN_W-1:0]      scan_wdata,

  // Voxel map write port: word k = {valid, nz, ny, nx, cz, cy, cx}.
  input  logic                   map_we,
  input  logic [MAP_AW-1:0]      map_waddr,
  input  logic [MAP_W-1:0]       map_wdata,

  // Control.
  input  logic                   start,
  input  logic [31:0]            num_points,
  input  logic [9*NORMAL_W-1:0]  pose_r,       // row-major R, element i at [i*NORMAL_W +: NORMAL_W]
  input  logic [3*COORD_W-1:0]   pose_t,       // t, element i at [i*COORD_W +: COORD_W]
  output logic                   busy,
  output logic                   done,         // level: result valid, cleared by start

  // Result, accum_t raw bits.
  output logic [21*ACCUM_W-1:0]  h_flat,       // H[k] at [k*ACCUM_W +: ACCUM_W]
  output logic [6*ACCUM_W-1:0]   g_flat,
  output logic [ACCUM_W-1:0]     cost,
  output logic [31:0]            inlier_count,
  output logic [31:0]            processed_count,
  output logic [31:0]            rejected_count,
  output logic [31:0]            status
);
  // ---- Memories ----------------------------------------------------------------
  logic [SCAN_AW-1:0] scan_raddr;
  logic [SCAN_W-1:0]  scan_rdata;
  logic [MAP_AW-1:0]  map_raddr;
  logic [MAP_W-1:0]   map_rdata;

  voxlio_ram #(.W(SCAN_W), .DEPTH(MAX_POINTS), .AW(SCAN_AW)) u_scan (
    .clk(clk), .we(scan_we), .waddr(scan_waddr), .wdata(scan_wdata),
    .raddr(scan_raddr), .rdata(scan_rdata));

  voxlio_ram #(.W(MAP_W), .DEPTH(NUM_VOXELS), .AW(MAP_AW)) u_map (
    .clk(clk), .we(map_we), .waddr(map_waddr), .wdata(map_wdata),
    .raddr(map_raddr), .rdata(map_rdata));

  // ---- Datapath ------------------------------------------------------------------
  logic xf_valid_in, xf_valid_out;
  logic signed [COORD_W-1:0] px, py, pz;

  voxlio_transform u_transform (
    .clk(clk), .rst_n(rst_n), .valid_in(xf_valid_in),
    .qx(scan_rdata[0*COORD_W +: COORD_W]),
    .qy(scan_rdata[1*COORD_W +: COORD_W]),
    .qz(scan_rdata[2*COORD_W +: COORD_W]),
    .pose_r(pose_r), .pose_t(pose_t),
    .valid_out(xf_valid_out), .px(px), .py(py), .pz(pz));

  logic grid_valid, grid_in_grid;
  logic [IX_W-1:0] ix;
  logic [IY_W-1:0] iy;
  logic [IZ_W-1:0] iz;

  voxlio_grid u_grid (
    .clk(clk), .rst_n(rst_n), .valid_in(xf_valid_out), .px(px), .py(py), .pz(pz),
    .valid_out(grid_valid), .in_grid(grid_in_grid), .ix(ix), .iy(iy), .iz(iz));

  logic search_start, search_done, found;
  logic signed [COMPUTE_W-1:0] best_r;
  logic signed [NORMAL_W-1:0]  best_nx, best_ny, best_nz;

  voxlio_search u_search (
    .clk(clk), .rst_n(rst_n), .start(search_start),
    .px(px), .py(py), .pz(pz), .ix(ix), .iy(iy), .iz(iz),
    .map_raddr(map_raddr), .map_rdata(map_rdata),
    .done(search_done), .found(found), .best_r(best_r),
    .best_nx(best_nx), .best_ny(best_ny), .best_nz(best_nz));

  logic jac_valid_in, jac_valid_out;
  logic [6*COMPUTE_W-1:0] j_flat;

  voxlio_jacobian u_jacobian (
    .clk(clk), .rst_n(rst_n), .valid_in(jac_valid_in),
    .px(px), .py(py), .pz(pz), .nx(best_nx), .ny(best_ny), .nz(best_nz),
    .valid_out(jac_valid_out), .j_flat(j_flat));

  logic acc_clear, acc_valid;

  voxlio_accumulate u_accumulate (
    .clk(clk), .rst_n(rst_n), .clear(acc_clear), .valid(acc_valid),
    .j_flat(j_flat), .r(best_r),
    .h_flat(h_flat), .g_flat(g_flat), .cost(cost), .inlier_count(inlier_count));

  // ---- Control -------------------------------------------------------------------
  typedef enum logic [3:0] {
    S_IDLE,        // wait for start
    S_CHECK,       // num_points check; clears the accumulators
    S_FETCH,       // scan RAM address = point index
    S_XFORM,       // scan word available: feed the transform
    S_WAIT_GRID,   // transform + grid pipelines (5 cycles)
    S_SEARCH,      // candidate walk running
    S_DECIDE,      // found? inlier?
    S_WAIT_JAC,    // Jacobian pipeline (3 cycles)
    S_WAIT_ACC,    // accumulator pipeline (2 cycles)
    S_NEXT,        // advance to the next point
    S_FINISH       // status word
  } state_t;

  state_t state;
  logic [31:0] point;            // index of the point in flight
  logic        overflow;         // num_points > MAX_POINTS
  logic        out_of_grid;      // at least one point was rejected by the bounds check
  logic [1:0]  wait_count;

  logic inlier;
  always_comb inlier = found && (sat_abs(best_r) < THRESHOLD_RAW);

  always_comb begin
    scan_raddr   = point[SCAN_AW-1:0];
    xf_valid_in  = (state == S_XFORM);
    search_start = (state == S_SEARCH) && (wait_count == 2'd0);
    jac_valid_in = (state == S_DECIDE) && inlier;
    acc_valid    = (state == S_WAIT_JAC) && jac_valid_out;
    acc_clear    = (state == S_CHECK);
    busy         = (state != S_IDLE);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state           <= S_IDLE;
      point           <= '0;
      overflow        <= 1'b0;
      out_of_grid     <= 1'b0;
      wait_count      <= '0;
      processed_count <= '0;
      rejected_count  <= '0;
      status          <= '0;
      done            <= 1'b0;
    end else begin
      case (state)
        S_IDLE: begin
          if (start) begin
            state           <= S_CHECK;
            done            <= 1'b0;
            point           <= '0;
            overflow        <= (num_points > MAX_POINTS);
            out_of_grid     <= 1'b0;
            processed_count <= '0;
            rejected_count  <= '0;
            status          <= '0;
          end
        end

        S_CHECK: begin
          if (overflow || num_points == 32'd0) state <= S_FINISH;
          else                                 state <= S_FETCH;
        end

        S_FETCH: begin
          processed_count <= processed_count + 32'd1;
          state <= S_XFORM;
        end

        S_XFORM: begin
          state <= S_WAIT_GRID;
        end

        S_WAIT_GRID: begin
          if (grid_valid) begin
            if (grid_in_grid) begin
              state      <= S_SEARCH;
              wait_count <= '0;
            end else begin
              rejected_count <= rejected_count + 32'd1;
              out_of_grid    <= 1'b1;
              state          <= S_NEXT;
            end
          end
        end

        S_SEARCH: begin
          // wait_count 0: start pulse; then hold until the search reports done.
          if (wait_count == 2'd0)  wait_count <= 2'd1;
          else if (search_done)    state <= S_DECIDE;
        end

        S_DECIDE: begin
          if (inlier) begin
            state <= S_WAIT_JAC;
          end else begin
            rejected_count <= rejected_count + 32'd1;
            state          <= S_NEXT;
          end
        end

        S_WAIT_JAC: begin
          if (jac_valid_out) begin
            state      <= S_WAIT_ACC;
            wait_count <= '0;
          end
        end

        S_WAIT_ACC: begin
          wait_count <= wait_count + 2'd1;
          if (wait_count == 2'd1) state <= S_NEXT;
        end

        S_NEXT: begin
          point <= point + 32'd1;
          if (point + 32'd1 == num_points) state <= S_FINISH;
          else                             state <= S_FETCH;
        end

        S_FINISH: begin
          status <= (overflow ? STATUS_POINT_OVERFLOW : 32'd0)
                  | (out_of_grid ? STATUS_OOB_PREVENTED : 32'd0)
                  | ((inlier_count == 32'd0) ? STATUS_ZERO_INLIERS : 32'd0)
                  | ((!overflow && inlier_count != 32'd0) ? STATUS_SUCCESS : 32'd0);
          done  <= 1'b1;
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end
endmodule
