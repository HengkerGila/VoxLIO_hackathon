// Stages 4 to 7: walk the (2R+1)^3 neighbourhood one candidate per cycle,
// read each descriptor, compute its signed residual and keep the usable
// candidate with the smallest |residual|. Candidates are judged strictly in
// dz, dy, dx order, so a tie keeps the first one, as in the C++ model.
//
// Pipeline (one candidate per cycle, issue at cycle n):
//   n    offsets, bounds check, map address      -> map_raddr
//   n+1  descriptor available, p - c
//   n+2  products n_a * (p_a - c_a)
//   n+3  sum
//   n+4  round / saturate to compute_t, usable flag
//   n+5  compare with the best so far, update

module voxlio_search
  import voxlio_pkg::*;
(
  input  logic                       clk,
  input  logic                       rst_n,
  input  logic                       start,      // pulse; inputs below stay stable until done
  input  logic signed [COORD_W-1:0]  px,
  input  logic signed [COORD_W-1:0]  py,
  input  logic signed [COORD_W-1:0]  pz,
  input  logic [IX_W-1:0]            ix,
  input  logic [IY_W-1:0]            iy,
  input  logic [IZ_W-1:0]            iz,
  output logic [MAP_AW-1:0]          map_raddr,
  input  logic [MAP_W-1:0]           map_rdata,  // one cycle after map_raddr
  output logic                       done,       // pulse, best_* valid from then on
  output logic                       found,
  output logic signed [COMPUTE_W-1:0] best_r,
  output logic signed [NORMAL_W-1:0] best_nx,
  output logic signed [NORMAL_W-1:0] best_ny,
  output logic signed [NORMAL_W-1:0] best_nz
);
  localparam int DIFF_W = COORD_W + 1;
  localparam int PROD_W = NORMAL_W + DIFF_W;            // COORD_F + NORMAL_F fraction bits
  localparam int SUM_W  = PROD_W + 2;
  localparam int DROP   = COORD_F + NORMAL_F - COMPUTE_F;
  localparam int OFF_W  = 4;                            // signed offset, radius up to 7
  localparam int CX_W   = IX_W + 2;                     // signed candidate coordinate
  localparam int CY_W   = IY_W + 2;
  localparam int CZ_W   = IZ_W + 2;

  // ---- Issue: nested offset counters ---------------------------------------
  logic                    issuing;
  logic signed [OFF_W-1:0] dx, dy, dz;
  logic                    last;                // this is the final candidate

  always_comb last = (dx == OFF_W'(NEIGHBOR_RADIUS)) && (dy == OFF_W'(NEIGHBOR_RADIUS))
                     && (dz == OFF_W'(NEIGHBOR_RADIUS));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      issuing <= 1'b0;
      dx <= '0;
      dy <= '0;
      dz <= '0;
    end else if (start) begin
      issuing <= 1'b1;
      dx <= -OFF_W'(NEIGHBOR_RADIUS);
      dy <= -OFF_W'(NEIGHBOR_RADIUS);
      dz <= -OFF_W'(NEIGHBOR_RADIUS);
    end else if (issuing) begin
      if (last) begin
        issuing <= 1'b0;
      end else if (dx == OFF_W'(NEIGHBOR_RADIUS)) begin
        dx <= -OFF_W'(NEIGHBOR_RADIUS);
        if (dy == OFF_W'(NEIGHBOR_RADIUS)) begin
          dy <= -OFF_W'(NEIGHBOR_RADIUS);
          dz <= dz + OFF_W'(1);
        end else begin
          dy <= dy + OFF_W'(1);
        end
      end else begin
        dx <= dx + OFF_W'(1);
      end
    end
  end

  // Candidate coordinates, bounds check (every axis, before any address is
  // formed) and address.
  logic signed [CX_W-1:0] cx;
  logic signed [CY_W-1:0] cy;
  logic signed [CZ_W-1:0] cz;
  logic                   in_grid;

  always_comb begin
    cx = CX_W'($signed({1'b0, ix})) + CX_W'(dx);
    cy = CY_W'($signed({1'b0, iy})) + CY_W'(dy);
    cz = CZ_W'($signed({1'b0, iz})) + CZ_W'(dz);
    in_grid = (cx >= 0) && (cx < CX_W'(NX)) && (cy >= 0) && (cy < CY_W'(NY))
              && (cz >= 0) && (cz < CZ_W'(NZ));
    // index = iz * NX * NY + iy * NX + ix, formed only from in-grid coordinates
    map_raddr = in_grid
              ? MAP_AW'((32'(cz[IZ_W-1:0]) * NY + 32'(cy[IY_W-1:0])) * NX + 32'(cx[IX_W-1:0]))
              : '0;
  end

  // ---- Pipeline registers ----------------------------------------------------
  // b: descriptor read in flight.  c: descriptor captured, p - c formed.
  // d: products.  e: sum.  f: residual quantised.
  logic b_valid, b_in_grid, b_last;
  logic c_valid, c_in_grid, c_last, c_flag;
  logic d_valid, d_in_grid, d_last, d_flag;
  logic e_valid, e_in_grid, e_last, e_flag;
  logic f_valid, f_usable, f_last;

  logic signed [NORMAL_W-1:0] c_n [3], d_n [3], e_n [3], f_n [3];
  logic signed [DIFF_W-1:0]   c_diff [3];
  logic signed [PROD_W-1:0]   d_prod [3];
  logic signed [SUM_W-1:0]    e_sum;
  logic signed [COMPUTE_W-1:0] f_r;

  // Descriptor word: {valid, nz, ny, nx, cz, cy, cx}.
  logic signed [COORD_W-1:0]  m_c [3];
  logic signed [NORMAL_W-1:0] m_n [3];
  logic                       m_flag;
  always_comb begin
    for (int a = 0; a < 3; a++) begin
      m_c[a] = map_rdata[a*COORD_W +: COORD_W];
      m_n[a] = map_rdata[3*COORD_W + a*NORMAL_W +: NORMAL_W];
    end
    m_flag = map_rdata[MAP_W-1];
  end

  logic signed [COORD_W-1:0] p [3];
  always_comb begin
    p[0] = px;
    p[1] = py;
    p[2] = pz;
  end

  logic signed [COMPUTE_W-1:0] e_rounded;
  voxlio_round_sat #(.IN_W(SUM_W), .DROP(DROP), .OUT_W(COMPUTE_W), .ROUND(ROUND_TO_NEAREST))
    u_round (.in(e_sum), .out(e_rounded));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      b_valid <= 1'b0;
      c_valid <= 1'b0;
      d_valid <= 1'b0;
      e_valid <= 1'b0;
      f_valid <= 1'b0;
    end else begin
      b_valid <= issuing;
      c_valid <= b_valid;
      d_valid <= c_valid;
      e_valid <= d_valid;
      f_valid <= e_valid;
    end
  end

  always_ff @(posedge clk) begin
    // b
    b_in_grid <= in_grid;
    b_last    <= last;
    // c
    c_in_grid <= b_in_grid;
    c_last    <= b_last;
    c_flag    <= m_flag;
    for (int a = 0; a < 3; a++) begin
      c_n[a]    <= m_n[a];
      c_diff[a] <= DIFF_W'(p[a]) - DIFF_W'(m_c[a]);
    end
    // d
    d_in_grid <= c_in_grid;
    d_last    <= c_last;
    d_flag    <= c_flag;
    for (int a = 0; a < 3; a++) begin
      d_n[a]    <= c_n[a];
      d_prod[a] <= c_n[a] * c_diff[a];
    end
    // e
    e_in_grid <= d_in_grid;
    e_last    <= d_last;
    e_flag    <= d_flag;
    for (int a = 0; a < 3; a++) e_n[a] <= d_n[a];
    e_sum <= SUM_W'(d_prod[0]) + SUM_W'(d_prod[1]) + SUM_W'(d_prod[2]);
    // f
    f_last   <= e_last;
    f_usable <= e_in_grid && e_flag && (e_n[0] != 0 || e_n[1] != 0 || e_n[2] != 0);
    for (int a = 0; a < 3; a++) f_n[a] <= e_n[a];
    f_r <= e_rounded;
  end

  // ---- Best-candidate tracker ------------------------------------------------
  logic signed [COMPUTE_W-1:0] best_abs;
  logic signed [COMPUTE_W-1:0] f_abs;
  always_comb f_abs = sat_abs(f_r);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      found    <= 1'b0;
      best_abs <= '0;
      best_r   <= '0;
      best_nx  <= '0;
      best_ny  <= '0;
      best_nz  <= '0;
      done     <= 1'b0;
    end else begin
      done <= f_valid && f_last;
      if (start) begin
        found    <= 1'b0;
        best_abs <= {1'b0, {(COMPUTE_W - 1){1'b1}}};   // compute_max()
        best_r   <= '0;
        best_nx  <= '0;
        best_ny  <= '0;
        best_nz  <= '0;
      end else if (f_valid && f_usable && (f_abs < best_abs)) begin
        found    <= 1'b1;
        best_abs <= f_abs;
        best_r   <= f_r;
        best_nx  <= f_n[0];
        best_ny  <= f_n[1];
        best_nz  <= f_n[2];
      end
    end
  end
endmodule
