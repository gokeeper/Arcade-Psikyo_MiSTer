// CRT Adjust chain: analog-CRT geometry between the core's native raster and
// arcade_video, built from rmonic79's two vendored modules:
//
//   native raster -> crt_vsize (V-Size) -> crt_adjust (H-Pos/V-Shift/H-Size)
//
// crt_vsize must sit ahead of crt_adjust (its integration guide), and
// everything downstream of it -- crt_adjust's write CE and its vb_in included
// -- has to come from crt_vsize's outputs, since it regenerates the line
// timing and the vertical window.
//
// With `active` low the whole chain is a registered passthrough at the core's
// own ce_pix: the output is the native raster, a few pixels late. HDMI taps
// the same output, so it follows the adjustment while `active` is high.
//
// What this module adds around the two vendored ones:
//  * the H-Size read-rate generator crt_adjust needs as its pxl2_cen;
//  * a VSync delay that anchors crt_vsize's PVM mode for Psikyo's blanking;
//  * two output fixups (VBlank on line boundaries, HSync pulse blanked).
// Each is explained where it is built.
//
// The inputs are already-decoded amounts; the OSD index decoding (list
// lengths, wrap points) belongs next to CONF_STR in Psikyo.sv.
module crt_chain #(
	parameter int HTOTAL  = 456,   // native pixels per line
	parameter int VTOTAL  = 262,   // native lines per frame
	parameter int ACTIVE_W = 320,  // active pixels per line
	// clk / pixel-CE ratio in quarter cycles: 85.909091 MHz / 7.159091 MHz
	// = 12 whole cycles = 48 quarters.
	parameter int RD_BASE = 48,
	// Lines after the last picture line that crt_vsize's PVM engine can drop
	// for free when it shortens the frame: the front porch (4) plus the line
	// VSync rises on, since the engine's frame starts on the line after it.
	parameter int PVM_FREE_LINES = 5
) (
	input  logic              clk,
	input  logic              ce_pix,

	input  logic              active,        // CRT Adjust On
	input  logic              scale_en,      // H-Size/V-Size allowed (native 15 kHz output)
	input  logic signed [8:0] hoffset,       // H-Position, pixels, + = right
	input  logic signed [5:0] voffset,       // V-Shift, lines
	input  logic signed [4:0] hsize,         // H-Size step, + = wider
	input  logic signed [3:0] vsize_step,    // V-Size step -7..+7, + = taller
	input  logic              vsize_cabinet, // V-Size mode: 0 = PVM, 1 = Cabinet

	input  logic [7:0]        r_in, g_in, b_in,
	input  logic              hs_in, vs_in, hb_in, vb_in,

	output logic              ce_out,        // sample the outputs on this CE
	output logic [7:0]        r_out, g_out, b_out,
	output logic              hs_out, vs_out, hb_out, vb_out
);

	wire scaling = active & scale_en;

	// ---- V-Size amount ----
	// One OSD step = 3 lines (~1.1%), so -7..+7 covers -21..+21 lines, which
	// the 46-line ring below holds. crt_vsize's convention is +N lines =
	// SHORTER picture, so the step is negated to make "+" mean taller.
	logic signed [5:0] vsize = 6'sd0;
	logic              cabinet = 1'b0;
	always_ff @(posedge clk) if (ce_pix) begin
		vsize   <= scaling ? -(6'sd3 * $signed({{2{vsize_step[3]}}, vsize_step})) : 6'sd0;
		cabinet <= vsize_cabinet;
	end

	// ---- PVM-mode anchor ----
	// The PVM engine starts its output frame on the line after VSync and,
	// when the picture is made taller (fewer, longer lines), drops the lines
	// left over at the END of the frame. That suits rasters whose blanking
	// sits mostly after the picture. Psikyo's is the other way round -- 33
	// lines from VSync to the first picture line, a 4-line front porch after
	// the last -- so past PVM_FREE_LINES every "taller" line would cut a
	// picture line off the bottom.
	//
	// Delaying the VSync the engine sees by D lines moves the dropped lines
	// into the back porch instead: the picture then ends D lines earlier in
	// the engine's frame and still fits while D >= -vsize - PVM_FREE_LINES.
	// D is one more than that, for a line of margin. The picture moves up by
	// D lines, so past the first step it grows mostly UPWARD -- recentre with
	// V-Shift. Cabinet mode keeps native timing and needs none of this.
	//
	// D is set from the requested vsize, not crt_vsize's internal 1-line-per-
	// frame ramp. Each change of D moves the VSync crt_vsize measures, so its
	// frame measurement is unstable for two frames and it passes the native
	// timing through for those frames before resuming. A brief jump per
	// "taller" click past the first, nothing more.
	localparam logic signed [6:0] PVM_MARGIN = 7'(PVM_FREE_LINES - 1);
	localparam int                PVM_DLY_MAX = 21 - (PVM_FREE_LINES - 1);

	logic                   hs_nat_d = 1'b0;
	logic [PVM_DLY_MAX-1:0] vs_lines = '0;   // VSync delayed by 1..PVM_DLY_MAX lines
	always_ff @(posedge clk) if (ce_pix) begin
		hs_nat_d <= hs_in;
		if (hs_in & ~hs_nat_d) vs_lines <= {vs_lines[PVM_DLY_MAX-2:0], vs_in};
	end

	wire signed [6:0] pvm_need = -$signed({vsize[5], vsize}) - PVM_MARGIN;
	wire        [4:0] pvm_dly  = (!cabinet && pvm_need > 0) ? pvm_need[4:0] : 5'd0;
	wire              vs_pre   = (pvm_dly == 0) ? vs_in : vs_lines[pvm_dly - 1'd1];

	// ---- V-Size (rtl/video/crt_vsize.sv) ----
	// Self-measuring; a registered passthrough (one ce_pix of latency) while
	// V-Size is 0 or `scaling` is low. LINE_PX only has to hold the active
	// pixels (it stores DE pixels), so ACTIVE_W rather than upstream's 384.
	logic [7:0] vz_r, vz_g, vz_b;
	logic       vz_hs, vz_vs, vz_de, vz_vb, vz_ce;

	crt_vsize #(
		.RING_LINES(46),
		.LINE_PX   (ACTIVE_W)
	) u_crt_vsize (
		.clk      (clk),
		.pxl_cen  (ce_pix),
		.active   (scaling),
		.tube_mode(cabinet),
		.vsize    (vsize),
		.r_in(r_in), .g_in(g_in), .b_in(b_in),
		.hs_in(hs_in), .vs_in(vs_pre),
		.de_in(~(hb_in | vb_in)),
		.vb_in(vb_in),            // TRUE vertical blank, never the combined one
		.r_out(vz_r), .g_out(vz_g), .b_out(vz_b),
		.hs_out(vz_hs), .vs_out(vz_vs), .de_out(vz_de), .vb_out(vz_vb),
		.ce_out(vz_ce)
	);

	// ---- H-Size read-rate generator ----
	// Read one pixel every (RD_BASE + hsize) QUARTERS of clk: hsize -16..+15
	// gives 32..63 quarters, each step ~2%, and hsize 0 reads at the write
	// rate. The accumulator restarts on the rise of crt_adjust's hs_ref_out,
	// NOT on the raw HSync: that shared edge keeps the write side, the
	// module's read counter and this read rate in phase (its documented
	// wiring rule).
	wire signed [4:0] hsize_eff = scaling ? hsize : 5'sd0;
	wire        [7:0] rd_period = 8'(RD_BASE) + {{3{hsize_eff[4]}}, hsize_eff};

	logic       hs_ref;
	logic       hs_ref_d = 1'b0;
	logic [7:0] rd_acc = 8'd0;
	wire        rd_tick = (rd_acc + 8'd4) >= rd_period;
	always_ff @(posedge clk) begin
		hs_ref_d <= hs_ref;
		if      (hs_ref & ~hs_ref_d) rd_acc <= 8'd0;
		else if (rd_tick)            rd_acc <= rd_acc + 8'd4 - rd_period;
		else                         rd_acc <= rd_acc + 8'd4;
	end

	// Off: crt_adjust is a passthrough clocked by its write CE, so everything
	// downstream keeps running on that CE.
	assign ce_out = active ? rd_tick : vz_ce;

	// ---- H-Position / V-Shift / H-Size (rtl/video/crt_adjust.sv) ----
	logic adj_hb, adj_vb;

	crt_adjust #(
		.VTOTAL   (VTOTAL),
		.HTOTAL   (HTOTAL),
		// CONTENTSHIFT keeps HSync byte-for-byte native (SYNCSHIFT moves the
		// sync itself); the mode Raiden ships and the safer one for sync lock.
		.HPOS_MODE(1)
	) u_crt_adjust (
		.clk      (clk),
		.pxl_cen  (vz_ce),       // write rate: crt_vsize's (possibly retimed) CE
		.pxl2_cen (rd_tick),     // read rate: the H-Size generator above
		.active   (active),
		.hsize    (hsize_eff),
		.hoffset  (active ? hoffset : 9'sd0),
		.voffset  (active ? voffset : 6'sd0),
		.r_in(vz_r), .g_in(vz_g), .b_in(vz_b),
		// crt_vsize regenerates the vertical window, so its OWN vb goes on --
		// the native vblank would black out the extra rows of a taller
		// picture.
		.hs_in(vz_hs), .vs_in(vz_vs), .hb_in(~vz_de), .vb_in(vz_vb),
		.r_out(r_out), .g_out(g_out), .b_out(b_out),
		.hs_out(hs_out), .vs_out(vs_out), .hb_out(adj_hb), .vb_out(adj_vb),
		.hs_ref_out(hs_ref)
	);

	// ---- Output fixups (active only; Off is an untouched passthrough) ----
	// VBlank: crt_adjust emits each line one line late (it reads the previous
	// line out of its ping-pong buffer) but passes VBlank through from the
	// write side, so its vb_out rises before the last picture line has been
	// shown. arcade_video latches VBlank when HBlank falls, i.e. at the start
	// of each line's picture, and by then it is already high: the last line
	// is dropped. Latching it at the output's own HSync instead -- before the
	// line's picture starts -- keeps that line.
	//
	// HBlank: when H-Size and H-Position push the picture's right edge past
	// the next HSync, the first read tick after HSync still carries the
	// previous line's last pixel, so it comes out as a one-pixel "active" stub
	// inside the sync pulse. Nothing inside HSync can be seen on a tube (the
	// beam is retracing), but the stub would open DE for one pixel, which the
	// scaler and the HDMI rotator's width measurement would see. The sync
	// pulse is blanked outright.
	logic hs_out_d = 1'b0, vb_line = 1'b1;
	always_ff @(posedge clk) if (ce_out) begin
		hs_out_d <= hs_out;
		if (hs_out & ~hs_out_d) vb_line <= adj_vb;
	end

	assign vb_out = active ? vb_line : adj_vb;
	assign hb_out = active ? (adj_hb | hs_out) : adj_hb;

endmodule
