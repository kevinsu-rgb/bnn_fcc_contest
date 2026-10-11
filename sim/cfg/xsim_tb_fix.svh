// Included inside every cfg_<name> wrapper, after the `bnn_fcc_tb ... tb ();` instance.
// xsim 2025.2 resizes a dynamic array to 0 in std::randomize(arr) (repro: v = new[5];
// std::randomize(v) -> v.size() == 0). In custom-topology mode the provided TB therefore builds a model
// whose weight rows are empty (every weight reads 0) and random images that are empty (no beats are
// sent, the run times out). This block runs at time 1, before the TB starts streaming, and refills both
// through the TB's own public API: random weight rows in tb.model.weight, re-encoded configuration
// stream, and random images loaded with BNN_FCC_Stimulus::load_from_file. Thresholds (scalar
// randomize) are kept. The TB file itself is not modified.
// xsim also returns 0 for .sum() of a real queue, so the TB's "Avg latency" line always reads 0.0 under
// xsim. The monitor below measures the same quantity (posedge of an image's first accepted beat to the
// posedge of its output handshake, in cycles) and prints it at the end of the run.
initial begin : xsim_tb_fix
  int    fd, f0, n_img;
  string path;
  #1;
  for (int l = 0; l < tb.model.num_layers; l++)
    for (int n = 0; n < tb.model.topology[l+1]; n++) begin
      tb.model.weight[l][n] = new[tb.model.topology[l]];
      for (int i = 0; i < tb.model.topology[l]; i++) tb.model.weight[l][n][i] = 1'($urandom);
    end
  tb.model.encode_configuration(tb.config_bus_data_stream, tb.config_bus_keep_stream);

  f0    = tb.model.topology[0];
  n_img = tb.NUM_TEST_IMAGES;
  path  = "xsim_tb_fix_images.hex";
  fd    = $fopen(path, "w");
  if (fd == 0) $fatal(1, "xsim_tb_fix: cannot open %s", path);
  for (int k = 0; k < n_img; k++) begin
    for (int p = 0; p < f0; p++) $fwrite(fd, "%02h", 8'($urandom));
    $fwrite(fd, "\n");
  end
  $fclose(fd);
  tb.stim.load_from_file(path, n_img);
  if (tb.stim.get_num_vectors() != n_img) $fatal(1, "xsim_tb_fix: loaded %0d images", tb.stim.get_num_vectors());
end

longint lat_cyc = 0, lat_sum = 0;
int     lat_n = 0;
bit     lat_mid = 1'b0;
longint lat_start [$];

always @(posedge tb.clk) begin
  lat_cyc++;
  if (tb.data_in.tvalid && tb.data_in.tready) begin
    if (!lat_mid) lat_start.push_back(lat_cyc);
    lat_mid = !tb.data_in.tlast;
  end
  if (tb.data_out.tvalid && tb.data_out.tready && lat_start.size() > 0) begin
    lat_sum += lat_cyc - lat_start.pop_front();
    lat_n++;
  end
end

final if (lat_n > 0)
  $display("xsim_tb_fix: Avg latency (cycles) per image: %0.1f (%0d images)", real'(lat_sum) / lat_n, lat_n);
