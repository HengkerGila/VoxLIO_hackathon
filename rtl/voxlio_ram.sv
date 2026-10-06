// Simple dual-port RAM: one write port (host), one read port (core), read
// data one cycle after the address. Quartus infers M10K blocks from this.

module voxlio_ram #(
  parameter int W     = 72,
  parameter int DEPTH = 32768,
  parameter int AW    = 15
) (
  input  logic          clk,
  input  logic          we,
  input  logic [AW-1:0] waddr,
  input  logic [W-1:0]  wdata,
  input  logic [AW-1:0] raddr,
  output logic [W-1:0]  rdata
);
  logic [W-1:0] mem [DEPTH];

  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end
endmodule
