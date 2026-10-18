// Memory wrapper used by every weight / threshold / activation memory.
//
//   PORTS = "SDP": simple dual port; write at wr_addr when we, independent read at rd_addr.
//   PORTS = "SP" : single port; rd_addr is the one shared address (wr_addr is ignored); a
//                  cycle with we=1 is a write, a cycle with we=0 is a read.
//
// Read data appears READ_LATENCY clock cycles after the address (read-before-write on a
// same-address SDP collision).
//   RAM_STYLE = "distributed": asynchronous array read, then READ_LATENCY output registers.
//   RAM_STYLE = "block"/"ultra": synchronous read is the first of the READ_LATENCY registers
//                  (so the primitive's own output register(s) can absorb the later stages).
module bnn_ram import bnn_fcc_pkg::*; #(
    parameter int    WIDTH        = 16,
    parameter int    DEPTH        = 98,
    parameter string RAM_STYLE    = "distributed",
    parameter int    READ_LATENCY = 1,
    parameter string PORTS        = "SP"
) (
    input  logic                     clk,
    input  logic                     we,
    // wr_addr is intentionally unused when PORTS="SP" (rd_addr is the shared address).
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [addr_w(DEPTH)-1:0] wr_addr,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [    WIDTH-1:0]     wr_data,
    input  logic [addr_w(DEPTH)-1:0] rd_addr,
    output logic [    WIDTH-1:0]     rd_data
);

  localparam bit IS_SDP = (PORTS == "SDP");
  // DEPTH=1 still has a 1-bit address; keep two words so the address bit stays in range.
  localparam int MEM_DEPTH = (DEPTH < 2) ? 2 : DEPTH;

  if (WIDTH < 1) begin : g_chk_width
    $error("bnn_ram: WIDTH must be >= 1 (got %0d)", WIDTH);
  end
  if (DEPTH < 1) begin : g_chk_depth
    $error("bnn_ram: DEPTH must be >= 1 (got %0d)", DEPTH);
  end
  if (READ_LATENCY < 1) begin : g_chk_lat
    $error("bnn_ram: READ_LATENCY must be >= 1 (got %0d)", READ_LATENCY);
  end
  if (!(RAM_STYLE == "distributed" || RAM_STYLE == "block" || RAM_STYLE == "ultra")) begin : g_chk_style
    $error("bnn_ram: RAM_STYLE must be distributed, block or ultra (got %s)", RAM_STYLE);
  end
  if (!(PORTS == "SP" || PORTS == "SDP")) begin : g_chk_ports
    $error("bnn_ram: PORTS must be SP or SDP (got %s)", PORTS);
  end

  // Shared address in SP mode, separate write address in SDP mode.
  logic [addr_w(DEPTH)-1:0] w_addr;
  if (IS_SDP) begin : g_sdp_addr
    assign w_addr = wr_addr;
  end else begin : g_sp_addr
    assign w_addr = rd_addr;
  end

  // Vivado rejects a string parameter inside the ram_style attribute, so the array is
  // declared once per style with a literal attribute (the same body via a local macro).
  logic [WIDTH-1:0] rd_raw;

`define BNN_RAM_ARRAY(STYLE) \
  (* ram_style = STYLE *) logic [WIDTH-1:0] mem[0:MEM_DEPTH-1]; \
  always_ff @(posedge clk) begin \
    if (we) mem[w_addr] <= wr_data; \
  end \
  assign rd_raw = mem[rd_addr];

  if (RAM_STYLE == "block") begin : g_block
    `BNN_RAM_ARRAY("block")
  end else if (RAM_STYLE == "ultra") begin : g_ultra
    `BNN_RAM_ARRAY("ultra")
  end else begin : g_dist
    `BNN_RAM_ARRAY("distributed")
  end
`undef BNN_RAM_ARRAY

  // Read pipeline. Stage 0 registers the array read: for "distributed" the array read is
  // asynchronous (LUTRAM) and this register follows it; for "block"/"ultra" this register is
  // the primitive's synchronous read. The RTL is identical, the ram_style picks the primitive.
  (* shreg_extract = "no" *) logic [WIDTH-1:0] rd_pipe[READ_LATENCY];

  always_ff @(posedge clk) begin
    rd_pipe[0] <= rd_raw;
    for (int i = 1; i < READ_LATENCY; i++) rd_pipe[i] <= rd_pipe[i-1];
  end

  assign rd_data = rd_pipe[READ_LATENCY-1];

endmodule
