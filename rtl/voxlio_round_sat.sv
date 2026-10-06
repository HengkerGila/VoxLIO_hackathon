// Quantise a wide fixed-point value to a narrower one the way ap_fixed does:
// drop DROP fraction bits with AP_RND (add half an LSB, then floor) or
// AP_TRN (floor), then AP_SAT to OUT_W bits. Purely combinational.

module voxlio_round_sat #(
  parameter int IN_W  = 48,
  parameter int DROP  = 16,
  parameter int OUT_W = 24,
  parameter bit ROUND = 1
) (
  input  logic signed [IN_W-1:0]  in,
  output logic signed [OUT_W-1:0] out
);
  localparam int RW = IN_W + 1;      // room for the carry of the half-LSB add
  localparam int SW = RW - DROP;     // bits that survive the shift

  localparam logic signed [RW-1:0] HALF =
      (ROUND && DROP > 0) ? (RW'(1) <<< (DROP - 1)) : RW'(0);

  /* verilator lint_off UNUSEDSIGNAL */
  logic signed [RW-1:0] rounded;     // its low DROP bits are discarded by design
  /* verilator lint_on UNUSEDSIGNAL */
  logic signed [SW-1:0] shifted;

  always_comb begin
    rounded = RW'(in) + HALF;
    shifted = rounded[RW-1:DROP];    // arithmetic shift: floor
  end

  generate
    if (SW > OUT_W) begin : g_sat
      // Overflow when the bits above the output's sign bit are not all copies of it.
      logic overflow;
      always_comb begin
        overflow = (shifted[SW-1:OUT_W-1] != {(SW - OUT_W + 1){shifted[SW-1]}});
        if (!overflow)          out = shifted[OUT_W-1:0];
        else if (shifted[SW-1]) out = {1'b1, {(OUT_W - 1){1'b0}}};  // most negative
        else                    out = {1'b0, {(OUT_W - 1){1'b1}}};  // most positive
      end
    end else begin : g_extend
      always_comb out = OUT_W'(shifted);
    end
  endgenerate
endmodule
