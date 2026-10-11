// System test: real trained MNIST model (python/model_data, python/test_vectors) through the full
// bnn_fcc DUT at the judged parallelism (PARALLEL_INPUTS=66, PARALLEL_NEURONS={32,11,1}).
// The provided model class is subclassed because under xsim its load_single_layer accumulates the
// per-line bit queue across lines (block-local queue in a loop is not re-created); the subclass
// declares the queue outside the loop and clears it each line. Latency/throughput are computed here
// with integer counters because xsim's .sum() on a real queue returns 0.
module tb_mnist;
  import bnn_fcc_tb_pkg::*;

  parameter string BASE_DIR = "../python";

  localparam int BUS_W      = 64;
  localparam int NUM_IMAGES = 100;
  localparam int F0         = 784;
  localparam int PPB        = BUS_W / 8;  // pixels per beat
  localparam real P_VALID   = 0.8;
  localparam real P_READY   = 0.7;

  class BNN_FCC_Model_XsimSafe #(
      int BUS_WIDTH = 64
  ) extends BNN_FCC_Model #(BUS_WIDTH);

    function void load_from_file(string data_dir, int expected_topology[]);
      this.topology   = expected_topology;
      this.num_layers = expected_topology.size() - 1;
      this.weight     = new[num_layers];
      this.threshold  = new[num_layers];
      for (int i = 0; i < num_layers; i++) begin
        string w_path = $sformatf("%s/l%0d_weights.txt", data_dir, i);
        string t_path = $sformatf("%s/l%0d_thresholds.txt", data_dir, i);
        load_single_layer(i, w_path, t_path);
      end
      is_loaded     = 1;
      outputs_valid = 0;
    endfunction

    protected function void load_single_layer(int l_idx, string w_file, string t_file);
      int          fd;
      string       line;
      weight_row_t w_queue[$];
      bit          bit_q[$];
      weight_row_t row;

      fd = $fopen(w_file, "r");
      if (fd == 0) $fatal(1, "BNN_FCC_Model_XsimSafe: Could not open %s", w_file);
      while ($fgets(line, fd)) begin
        bit_q.delete();
        for (int i = 0; i < line.len(); i++) begin
          if (line[i] == "1") bit_q.push_back(1'b1);
          else if (line[i] == "0") bit_q.push_back(1'b0);
        end
        if (bit_q.size() > 0) begin
          row = new[bit_q.size()];
          foreach (bit_q[k]) row[k] = bit_q[k];
          w_queue.push_back(row);
        end
      end
      $fclose(fd);

      this.weight[l_idx] = new[w_queue.size()];
      foreach (w_queue[i]) this.weight[l_idx][i] = w_queue[i];

      fd = $fopen(t_file, "r");
      if (fd == 0) $fatal(1, "BNN_FCC_Model_XsimSafe: Could not open %s", t_file);
      this.threshold[l_idx] = new[w_queue.size()];
      for (int i = 0; i < w_queue.size(); i++) void'($fscanf(fd, "%d\n", this.threshold[l_idx][i]));
      $fclose(fd);
    endfunction
  endclass

  // ---------------- DUT
  logic clk = 1'b0;
  logic rst = 1'b1;
  always #5 clk = ~clk;

  logic                config_valid = 1'b0, config_ready, config_last = 1'b0;
  logic [BUS_W-1:0]    config_data = '0;
  logic [BUS_W/8-1:0]  config_keep = '0;
  logic                data_in_valid = 1'b0, data_in_ready, data_in_last = 1'b0;
  logic [BUS_W-1:0]    data_in_data = '0;
  logic [BUS_W/8-1:0]  data_in_keep = '0;
  logic                data_out_valid, data_out_ready = 1'b0, data_out_last;
  logic [7:0]          data_out_data;
  logic [0:0]          data_out_keep;

  bnn_fcc #(
    .INPUT_DATA_WIDTH(8), .INPUT_BUS_WIDTH(BUS_W), .CONFIG_BUS_WIDTH(BUS_W),
    .OUTPUT_DATA_WIDTH(4), .OUTPUT_BUS_WIDTH(8),
    .TOTAL_LAYERS(4), .TOPOLOGY('{784, 256, 256, 10}),
    .PARALLEL_INPUTS(66), .PARALLEL_NEURONS('{32, 11, 1})
  ) dut (.*);

  // ---------------- TB state
  BNN_FCC_Model_XsimSafe #(BUS_W) model;
  BNN_FCC_Stimulus #(.PIXEL_WIDTH(8)) stim;
  BNN_FCC_Model_XsimSafe #(BUS_W)::bus_stream_t  cfg_data;
  BNN_FCC_Model_XsimSafe #(BUS_W)::keep_stream_t cfg_keep;
  int expected[NUM_IMAGES];
  int topo[];
  int errors = 0;
  int out_count = 0;

  longint cyc = 0;
  longint start_cyc[NUM_IMAGES];
  longint lat_sum = 0;
  longint first_start = -1, last_out = 0;
  int     started = 0;
  bit     in_mid = 1'b0;

  always @(posedge clk) begin
    cyc <= cyc + 1;
  end

  // Input-side start tracking: first accepted beat of each image.
  always @(posedge clk) if (!rst) begin
    if (data_in_valid && data_in_ready) begin
      if (!in_mid && started < NUM_IMAGES) begin
        start_cyc[started] = cyc;
        if (started == 0) first_start = cyc;
        started++;
      end
      in_mid = !data_in_last;
    end
  end

  // Output monitor + ready randomization
  always @(posedge clk) begin
    if (rst) data_out_ready <= 1'b0;
    else begin
      if (data_out_valid && data_out_ready) begin
        if (out_count >= NUM_IMAGES) begin
          $error("tb_mnist: unexpected extra output");
        end else begin
          if (data_out_data !== 8'(expected[out_count])) begin
            $error("tb_mnist: image %0d: got %0d expected %0d", out_count, data_out_data, expected[out_count]);
            errors++;
          end
          if (data_out_keep !== 1'b1 || data_out_last !== 1'b1) begin
            $error("tb_mnist: image %0d: keep=%b last=%b", out_count, data_out_keep, data_out_last);
            errors++;
          end
          if (out_count < started) lat_sum += cyc - start_cyc[out_count];
          else begin
            $error("tb_mnist: output %0d before its image was accepted", out_count);
            errors++;
          end
          last_out = cyc;
        end
        out_count++;
      end
      data_out_ready <= (real'($urandom_range(999)) / 1000.0) < P_READY;
    end
  end

  // Watchdog
  initial begin
    #(10ns * 5000000);
    $fatal(1, "tb_mnist: TIMEOUT (outputs %0d/%0d)", out_count, NUM_IMAGES);
  end

  function automatic bit gap_ok();
    return (real'($urandom_range(999)) / 1000.0) < P_VALID;
  endfunction

  // ---------------- main
  int fd, code;
  bit [7:0] img[];
  bit [7:0] img_vec[];
  int nbeats;

  initial begin
    string ref_dir, vec_file;
    ref_dir  = {BASE_DIR, "/model_data"};
    vec_file = {BASE_DIR, "/test_vectors/inputs.hex"};
    topo = '{784, 256, 256, 10};

    model = new();
    model.load_from_file(ref_dir, topo);
    stim = new(F0);
    stim.load_from_file(vec_file, NUM_IMAGES);
    if (stim.get_num_vectors() != NUM_IMAGES) $fatal(1, "tb_mnist: loaded %0d images", stim.get_num_vectors());

    fd = $fopen({BASE_DIR, "/test_vectors/expected_outputs.txt"}, "r");
    if (fd == 0) $fatal(1, "tb_mnist: cannot open expected_outputs.txt");
    for (int i = 0; i < NUM_IMAGES; i++) begin
      code = $fscanf(fd, "%d\n", expected[i]);
      if (code != 1) $fatal(1, "tb_mnist: bad expected_outputs.txt line %0d", i);
    end
    $fclose(fd);

    // Prove the loader: reference model must match python on all images.
    for (int i = 0; i < NUM_IMAGES; i++) begin
      int pred;
      stim.get_vector(i, img_vec);
      pred = model.compute_reference(img_vec);
      if (pred != expected[i]) $fatal(1, "tb_mnist: reference model mismatch image %0d: %0d vs %0d", i, pred, expected[i]);
    end
    $display("tb_mnist: reference model matches expected_outputs for %0d images", NUM_IMAGES);

    model.encode_configuration(cfg_data, cfg_keep);

    repeat (5) @(posedge clk);
    rst <= 1'b0;
    repeat (3) @(posedge clk);

    // Configuration
    for (int i = 0; i < cfg_data.size(); i++) begin
      while (!gap_ok()) begin
        config_valid <= 1'b0;
        @(posedge clk);
      end
      config_valid <= 1'b1;
      config_data  <= cfg_data[i];
      config_keep  <= cfg_keep[i];
      config_last  <= (i == cfg_data.size() - 1);
      do @(posedge clk); while (!config_ready);
    end
    config_valid <= 1'b0;
    config_last  <= 1'b0;
    $display("tb_mnist: configuration sent (%0d beats)", cfg_data.size());

    // Wait until configured
    while (!data_in_ready) @(posedge clk);

    // Images
    nbeats = (F0 + PPB - 1) / PPB;
    for (int n = 0; n < NUM_IMAGES; n++) begin
      stim.get_vector(n, img);
      for (int b = 0; b < nbeats; b++) begin
        while (!gap_ok()) begin
          data_in_valid <= 1'b0;
          @(posedge clk);
        end
        data_in_valid <= 1'b1;
        data_in_data  <= '0;
        data_in_keep  <= '0;
        for (int k = 0; k < PPB; k++) begin
          if (b * PPB + k < F0) begin
            data_in_data[k*8+:8] <= img[b*PPB+k];
            data_in_keep[k]      <= 1'b1;
          end
        end
        data_in_last <= (b == nbeats - 1);
        do @(posedge clk); while (!data_in_ready);
      end
    end
    data_in_valid <= 1'b0;
    data_in_last  <= 1'b0;

    while (out_count < NUM_IMAGES) @(posedge clk);
    repeat (20) @(posedge clk);

    if (out_count != NUM_IMAGES) errors++;
    $display("tb_mnist: Avg latency (cycles) per image: %0.1f", real'(lat_sum) / NUM_IMAGES);
    $display("tb_mnist: Avg throughput (cycles/output): %0.1f", real'(last_out - first_start) / NUM_IMAGES);
    if (errors == 0) begin
      $display("MNIST: %0d/%0d match python", NUM_IMAGES, NUM_IMAGES);
      $display("SUCCESS: tb_mnist");
    end else $display("tb_mnist: %0d errors", errors);
    $finish;
  end
endmodule
