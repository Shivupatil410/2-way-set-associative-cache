# 2-way-set-associative-cache
//==============================================================================
// cache_soc.v  -  Plain Verilog-2001, synthesizable (NO simulation code here)
//
//   l1_icache    2-way, read-only instruction cache   (256 B)
//   l1_dcache    2-way, write-back data cache         (256 B)
//   l2_cache     2-way unified cache                  (1 KB)
//   cache_system = L1 I + L1 D + arbiter + L2         <-- SYNTHESIZE THIS FIRST
//                  (no processor needed; simple req/ready ports)
//   soc_top      = CV32E40P core + connector logic + cache_system   (optional)
//
// Line = 16 bytes.  Address split:  | TAG | INDEX | WORD(2) | BYTE(2) |
// Handshake used by every cache:  hold req (and address/data) steady until
// ready=1.  When ready=1 the access is finished in that clock cycle.
//==============================================================================


//==============================================================================
// L1 INSTRUCTION CACHE  (2-way, LRU, read-only)
// cpu_req must stay high until cpu_ready. Miss address is latched, so the fill
// is correct even if the core changes the fetch address during the miss.
//==============================================================================
module l1_icache #(parameter INDEX_BITS = 3)(
    input          clk,
    input          rst_n,
    // CPU side
    input          cpu_req,
    input  [31:0]  cpu_addr,
    output [31:0]  cpu_rdata,
    output         cpu_ready,
    // L2 side (whole line)
    output         l2_req,
    output [31:0]  l2_addr,
    input  [127:0] l2_rdata,
    input          l2_ready
);
    localparam NSETS    = 1 << INDEX_BITS;
    localparam TAG_BITS = 32 - INDEX_BITS - 4;
    localparam IDLE = 1'b0, FILL = 1'b1;

    reg                state;
    reg [127:0]        data0 [0:NSETS-1];
    reg [127:0]        data1 [0:NSETS-1];
    reg [TAG_BITS-1:0] tags0 [0:NSETS-1];
    reg [TAG_BITS-1:0] tags1 [0:NSETS-1];
    reg [NSETS-1:0]    valid0, valid1;
    reg [NSETS-1:0]    lru;          // lru[i] = way to replace next in set i
    reg [31:0]         miss_addr;

    wire [1:0]            word_sel = cpu_addr[3:2];
    wire [INDEX_BITS-1:0] index    = cpu_addr[4 +: INDEX_BITS];
    wire [TAG_BITS-1:0]   tag      = cpu_addr[4+INDEX_BITS +: TAG_BITS];

    wire [INDEX_BITS-1:0] m_index  = miss_addr[4 +: INDEX_BITS];
    wire [TAG_BITS-1:0]   m_tag    = miss_addr[4+INDEX_BITS +: TAG_BITS];

    wire hit0 = valid0[index] && (tags0[index] == tag);
    wire hit1 = valid1[index] && (tags1[index] == tag);
    wire hit  = hit0 | hit1;

    // victim for fill: empty way first, else LRU way
    wire victim = !valid0[m_index] ? 1'b0 :
                  !valid1[m_index] ? 1'b1 : lru[m_index];

    wire [127:0] line = hit1 ? data1[index] : data0[index];
    assign cpu_rdata = line[word_sel*32 +: 32];
    assign cpu_ready = (state == IDLE) && cpu_req && hit;

    assign l2_req  = (state == FILL);
    assign l2_addr = {miss_addr[31:4], 4'b0000};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            valid0    <= {NSETS{1'b0}};
            valid1    <= {NSETS{1'b0}};
            lru       <= {NSETS{1'b0}};
            miss_addr <= 32'b0;
        end else begin
            case (state)
                IDLE: if (cpu_req) begin
                    if (hit) lru[index] <= hit0;       // way0 used -> way1 is LRU
                    else begin
                        miss_addr <= cpu_addr;
                        state     <= FILL;
                    end
                end
                FILL: if (l2_ready) begin
                    if (victim) valid1[m_index] <= 1'b1;
                    else        valid0[m_index] <= 1'b1;
                    lru[m_index] <= ~victim;
                    state        <= IDLE;
                end
            endcase
        end
    end

    // data/tag arrays (no reset needed; valid bits protect them)
    always @(posedge clk) begin
        if (state == FILL && l2_ready) begin
            if (victim) begin data1[m_index] <= l2_rdata; tags1[m_index] <= m_tag; end
            else        begin data0[m_index] <= l2_rdata; tags0[m_index] <= m_tag; end
        end
    end
endmodule


//==============================================================================
// L1 DATA CACHE  (2-way, LRU, write-back, write-allocate, byte strobes)
// A store hit is written at the clock edge where cpu_ready=1.
//==============================================================================
module l1_dcache #(parameter INDEX_BITS = 3)(
    input          clk,
    input          rst_n,
    // CPU side
    input          cpu_req,
    input          cpu_we,
    input  [3:0]   cpu_wstrb,
    input  [31:0]  cpu_addr,
    input  [31:0]  cpu_wdata,
    output [31:0]  cpu_rdata,
    output         cpu_ready,
    // L2 side (whole line)
    output         l2_req,
    output         l2_we,
    output [31:0]  l2_addr,
    output [127:0] l2_wdata,
    input  [127:0] l2_rdata,
    input          l2_ready
);
    localparam NSETS    = 1 << INDEX_BITS;
    localparam TAG_BITS = 32 - INDEX_BITS - 4;
    localparam IDLE = 2'd0, WB = 2'd1, FILL = 2'd2;

    reg [1:0]          state;
    reg [127:0]        data0 [0:NSETS-1];
    reg [127:0]        data1 [0:NSETS-1];
    reg [TAG_BITS-1:0] tags0 [0:NSETS-1];
    reg [TAG_BITS-1:0] tags1 [0:NSETS-1];
    reg [NSETS-1:0]    valid0, valid1, dirty0, dirty1, lru;

    wire [1:0]            word_sel = cpu_addr[3:2];
    wire [INDEX_BITS-1:0] index    = cpu_addr[4 +: INDEX_BITS];
    wire [TAG_BITS-1:0]   tag      = cpu_addr[4+INDEX_BITS +: TAG_BITS];

    wire hit0 = valid0[index] && (tags0[index] == tag);
    wire hit1 = valid1[index] && (tags1[index] == tag);
    wire hit  = hit0 | hit1;

    wire victim = !valid0[index] ? 1'b0 :
                  !valid1[index] ? 1'b1 : lru[index];
    wire victim_dirty = victim ? (valid1[index] && dirty1[index])
                               : (valid0[index] && dirty0[index]);
    wire [127:0]        victim_line = victim ? data1[index] : data0[index];
    wire [TAG_BITS-1:0] victim_tag  = victim ? tags1[index] : tags0[index];

    wire [127:0] line = hit1 ? data1[index] : data0[index];

    // line after a store (only strobed bytes change)
    reg [127:0] new_line;
    integer b;
    always @(*) begin
        new_line = line;
        for (b = 0; b < 4; b = b + 1)
            if (cpu_wstrb[b])
                new_line[word_sel*32 + b*8 +: 8] = cpu_wdata[b*8 +: 8];
    end

    assign cpu_rdata = line[word_sel*32 +: 32];
    assign cpu_ready = (state == IDLE) && cpu_req && hit;

    assign l2_req   = (state == WB) || (state == FILL);
    assign l2_we    = (state == WB);
    assign l2_addr  = (state == WB) ? {victim_tag, index, 4'b0000}
                                    : {tag,        index, 4'b0000};
    assign l2_wdata = victim_line;

    wire hit_wr  = (state == IDLE) && cpu_req && cpu_we && hit;
    wire fill_we = (state == FILL) && l2_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= IDLE;
            valid0 <= {NSETS{1'b0}};  valid1 <= {NSETS{1'b0}};
            dirty0 <= {NSETS{1'b0}};  dirty1 <= {NSETS{1'b0}};
            lru    <= {NSETS{1'b0}};
        end else begin
            case (state)
                IDLE: if (cpu_req) begin
                    if (hit) begin
                        lru[index] <= hit0;
                        if (cpu_we) begin
                            if (hit1) dirty1[index] <= 1'b1;
                            else      dirty0[index] <= 1'b1;
                        end
                    end else
                        state <= victim_dirty ? WB : FILL;
                end
                WB: if (l2_ready) state <= FILL;
                FILL: if (l2_ready) begin
                    if (victim) begin valid1[index] <= 1'b1; dirty1[index] <= 1'b0; end
                    else        begin valid0[index] <= 1'b1; dirty0[index] <= 1'b0; end
                    lru[index] <= ~victim;
                    state      <= IDLE;        // replay -> now a hit
                end
                default: state <= IDLE;
            endcase
        end
    end

    always @(posedge clk) begin
        if (hit_wr) begin
            if (hit1) data1[index] <= new_line;
            else      data0[index] <= new_line;
        end
        if (fill_we) begin
            if (victim) begin data1[index] <= l2_rdata; tags1[index] <= tag; end
            else        begin data0[index] <= l2_rdata; tags0[index] <= tag; end
        end
    end
endmodule


//==============================================================================
// UNIFIED L2 CACHE (2-way, LRU, write-back, write-allocate)
// Requests are whole lines. we=1 -> write a full line (L1 write-back), so a
// write miss needs no read from main memory.
//==============================================================================
module l2_cache #(parameter INDEX_BITS = 5)(
    input          clk,
    input          rst_n,
    // L1 side (through arbiter)
    input          req,
    input          we,
    input  [31:0]  addr,
    input  [127:0] wdata,
    output [127:0] rdata,
    output         ready,
    // main memory side
    output         mem_req,
    output         mem_we,
    output [31:0]  mem_addr,
    output [127:0] mem_wdata,
    input  [127:0] mem_rdata,
    input          mem_ready
);
    localparam NSETS    = 1 << INDEX_BITS;
    localparam TAG_BITS = 32 - INDEX_BITS - 4;
    localparam IDLE = 2'd0, WB = 2'd1, FILL = 2'd2;

    reg [1:0]          state;
    reg [127:0]        data0 [0:NSETS-1];
    reg [127:0]        data1 [0:NSETS-1];
    reg [TAG_BITS-1:0] tags0 [0:NSETS-1];
    reg [TAG_BITS-1:0] tags1 [0:NSETS-1];
    reg [NSETS-1:0]    valid0, valid1, dirty0, dirty1, lru;

    wire [INDEX_BITS-1:0] index = addr[4 +: INDEX_BITS];
    wire [TAG_BITS-1:0]   tag   = addr[4+INDEX_BITS +: TAG_BITS];

    wire hit0 = valid0[index] && (tags0[index] == tag);
    wire hit1 = valid1[index] && (tags1[index] == tag);
    wire hit  = hit0 | hit1;

    wire victim = !valid0[index] ? 1'b0 :
                  !valid1[index] ? 1'b1 : lru[index];
    wire victim_dirty = victim ? (valid1[index] && dirty1[index])
                               : (valid0[index] && dirty0[index]);
    wire [127:0]        victim_line = victim ? data1[index] : data0[index];
    wire [TAG_BITS-1:0] victim_tag  = victim ? tags1[index] : tags0[index];

    assign rdata = hit1 ? data1[index] : data0[index];
    assign ready = (state == IDLE) && req && hit;

    assign mem_req   = (state == WB) || ((state == FILL) && !we);
    assign mem_we    = (state == WB);
    assign mem_addr  = (state == WB) ? {victim_tag, index, 4'b0000}
                                     : {tag,        index, 4'b0000};
    assign mem_wdata = victim_line;

    wire hit_wr  = (state == IDLE) && req && we && hit;
    wire fill_we = (state == FILL) && (we || mem_ready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= IDLE;
            valid0 <= {NSETS{1'b0}};  valid1 <= {NSETS{1'b0}};
            dirty0 <= {NSETS{1'b0}};  dirty1 <= {NSETS{1'b0}};
            lru    <= {NSETS{1'b0}};
        end else begin
            case (state)
                IDLE: if (req) begin
                    if (hit) begin
                        lru[index] <= hit0;
                        if (we) begin
                            if (hit1) dirty1[index] <= 1'b1;
                            else      dirty0[index] <= 1'b1;
                        end
                    end else
                        state <= victim_dirty ? WB : FILL;
                end
                WB: if (mem_ready) state <= FILL;
                FILL: if (we || mem_ready) begin
                    // a write replays as a hit next cycle and sets dirty then
                    if (victim) begin valid1[index] <= 1'b1; dirty1[index] <= 1'b0; end
                    else        begin valid0[index] <= 1'b1; dirty0[index] <= 1'b0; end
                    lru[index] <= ~victim;
                    state      <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end

    always @(posedge clk) begin
        if (hit_wr) begin
            if (hit1) data1[index] <= wdata;
            else      data0[index] <= wdata;
        end
        if (fill_we) begin
            if (victim) begin data1[index] <= we ? wdata : mem_rdata; tags1[index] <= tag; end
            else        begin data0[index] <= we ? wdata : mem_rdata; tags0[index] <= tag; end
        end
    end
endmodule


//==============================================================================
// CACHE SYSTEM : split L1 (I + D)  ->  arbiter  ->  unified L2  ->  main memory
// This is the whole project in one module. No processor inside.
//==============================================================================
module cache_system #(
    parameter L1_INDEX_BITS = 3,     // 8 sets  x 2 ways x 16 B = 256 B
    parameter L2_INDEX_BITS = 5      // 32 sets x 2 ways x 16 B = 1 KB
)(
    input          clk,
    input          rst_n,
    // instruction port (from processor fetch)
    input          i_req,
    input  [31:0]  i_addr,
    output [31:0]  i_rdata,
    output         i_ready,
    // data port (from processor load/store)
    input          d_req,
    input          d_we,
    input  [3:0]   d_wstrb,          // byte enables: sb / sh / sw
    input  [31:0]  d_addr,
    input  [31:0]  d_wdata,
    output [31:0]  d_rdata,
    output         d_ready,
    // main memory port (whole 128-bit lines)
    output         mem_req,
    output         mem_we,
    output [31:0]  mem_addr,
    output [127:0] mem_wdata,
    input  [127:0] mem_rdata,
    input          mem_ready
);
    wire         il2_req, il2_ready;
    wire [31:0]  il2_addr;
    wire         dl2_req, dl2_we, dl2_ready;
    wire [31:0]  dl2_addr;
    wire [127:0] dl2_wdata;
    wire         l2_req, l2_we, l2_ready;
    wire [31:0]  l2_addr;
    wire [127:0] l2_wdata, l2_rdata;

    l1_icache #(.INDEX_BITS(L1_INDEX_BITS)) u_l1i (
        .clk(clk), .rst_n(rst_n),
        .cpu_req(i_req), .cpu_addr(i_addr),
        .cpu_rdata(i_rdata), .cpu_ready(i_ready),
        .l2_req(il2_req), .l2_addr(il2_addr),
        .l2_rdata(l2_rdata), .l2_ready(il2_ready));

    l1_dcache #(.INDEX_BITS(L1_INDEX_BITS)) u_l1d (
        .clk(clk), .rst_n(rst_n),
        .cpu_req(d_req), .cpu_we(d_we), .cpu_wstrb(d_wstrb),
        .cpu_addr(d_addr), .cpu_wdata(d_wdata),
        .cpu_rdata(d_rdata), .cpu_ready(d_ready),
        .l2_req(dl2_req), .l2_we(dl2_we), .l2_addr(dl2_addr),
        .l2_wdata(dl2_wdata),
        .l2_rdata(l2_rdata), .l2_ready(dl2_ready));

    // ---- Arbiter: only one L1 talks to L2 at a time. D-cache wins a tie.
    //      The winner is "locked" until L2 says ready.
    localparam NONE = 2'd0, OWN_D = 2'd1, OWN_I = 2'd2;
    reg [1:0] owner;

    wire sel_d = (owner == OWN_D) || (owner == NONE && dl2_req);
    wire sel_i = (owner == OWN_I) || (owner == NONE && !dl2_req && il2_req);

    assign l2_req    = (sel_d && dl2_req) || (sel_i && il2_req);
    assign l2_we     = sel_d && dl2_we;
    assign l2_addr   = sel_i ? il2_addr : dl2_addr;
    assign l2_wdata  = dl2_wdata;             // I-cache never writes
    assign dl2_ready = l2_ready && sel_d;
    assign il2_ready = l2_ready && sel_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)             owner <= NONE;
        else if (l2_ready)      owner <= NONE;
        else if (owner == NONE) owner <= sel_d ? OWN_D : (sel_i ? OWN_I : NONE);
    end

    l2_cache #(.INDEX_BITS(L2_INDEX_BITS)) u_l2 (
        .clk(clk), .rst_n(rst_n),
        .req(l2_req), .we(l2_we), .addr(l2_addr), .wdata(l2_wdata),
        .rdata(l2_rdata), .ready(l2_ready),
        .mem_req(mem_req), .mem_we(mem_we), .mem_addr(mem_addr),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(mem_ready));
endmodule


//==============================================================================
// SOC TOP (OPTIONAL) : CV32E40P processor + connector logic + cache_system
// Needs the CV32E40P source files from github.com/openhwgroup/cv32e40p/rtl
// Addresses >= NC_BASE (peripherals) skip the cache and use the p_* port.
// If your core release has different port names, edit only the u_core block.
//==============================================================================
module soc_top #(
    parameter [31:0] BOOT_ADDR = 32'h0000_0000,
    parameter [31:0] NC_BASE   = 32'h1000_0000
)(
    input          clk,
    input          rst_n,
    input          fetch_enable,
    output         core_sleep,
    // main memory (whole lines)
    output         mem_req,
    output         mem_we,
    output [31:0]  mem_addr,
    output [127:0] mem_wdata,
    input  [127:0] mem_rdata,
    input          mem_ready,
    // peripheral port (not cached)
    output         p_req,
    output         p_we,
    output [3:0]   p_be,
    output [31:0]  p_addr,
    output [31:0]  p_wdata,
    input  [31:0]  p_rdata,
    input          p_ready
);
    wire        instr_req, instr_gnt;
    wire [31:0] instr_addr;
    reg         instr_rvalid;
    reg  [31:0] instr_rdata;
    wire        data_req, data_gnt, data_we;
    wire [3:0]  data_be;
    wire [31:0] data_addr, data_wdata;
    reg         data_rvalid;
    reg  [31:0] data_rdata;
    wire        ic_ready, dc_req, dc_ready;
    wire [31:0] ic_rdata, dc_rdata;

    // ---- the processor ----
    cv32e40p_core #(
        .COREV_PULP(0), .COREV_CLUSTER(0), .FPU(0),
        .FPU_ADDMUL_LAT(0), .FPU_OTHERS_LAT(0), .ZFINX(0),
        .NUM_MHPMCOUNTERS(1)
    ) u_core (
        .clk_i(clk), .rst_ni(rst_n),
        .pulp_clock_en_i(1'b1), .scan_cg_en_i(1'b0),
        .boot_addr_i(BOOT_ADDR), .mtvec_addr_i(BOOT_ADDR),
        .dm_halt_addr_i(32'h0000_0800), .hart_id_i(32'h0),
        .dm_exception_addr_i(32'h0000_0808),
        .instr_req_o(instr_req), .instr_gnt_i(instr_gnt),
        .instr_rvalid_i(instr_rvalid), .instr_addr_o(instr_addr),
        .instr_rdata_i(instr_rdata),
        .data_req_o(data_req), .data_gnt_i(data_gnt),
        .data_rvalid_i(data_rvalid), .data_we_o(data_we),
        .data_be_o(data_be), .data_addr_o(data_addr),
        .data_wdata_o(data_wdata), .data_rdata_i(data_rdata),
        .irq_i(32'b0), .irq_ack_o(), .irq_id_o(),
        .debug_req_i(1'b0), .debug_havereset_o(), .debug_running_o(),
        .debug_halted_o(),
        .fetch_enable_i(fetch_enable), .core_sleep_o(core_sleep)
    );

    // ---- connector for instructions ----
    // processor protocol: req -> gnt (accepted) -> rvalid + rdata one cycle later
    assign instr_gnt = instr_req && ic_ready;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin instr_rvalid <= 1'b0; instr_rdata <= 32'b0; end
        else begin
            instr_rvalid <= instr_gnt;
            if (instr_gnt) instr_rdata <= ic_rdata;
        end
    end

    // ---- connector for data (+ peripheral bypass) ----
    wire nc = (data_addr >= NC_BASE);
    assign dc_req  = data_req && !nc;
    assign p_req   = data_req &&  nc;
    assign p_we    = data_we;
    assign p_be    = data_be;
    assign p_addr  = data_addr;
    assign p_wdata = data_wdata;
    assign data_gnt = data_req && (nc ? p_ready : dc_ready);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin data_rvalid <= 1'b0; data_rdata <= 32'b0; end
        else begin
            data_rvalid <= data_gnt;
            if (data_gnt) data_rdata <= nc ? p_rdata : dc_rdata;
        end
    end

    // ---- the caches ----
    cache_system u_cache (
        .clk(clk), .rst_n(rst_n),
        .i_req(instr_req), .i_addr(instr_addr),
        .i_rdata(ic_rdata), .i_ready(ic_ready),
        .d_req(dc_req), .d_we(data_we), .d_wstrb(data_be),
        .d_addr(data_addr), .d_wdata(data_wdata),
        .d_rdata(dc_rdata), .d_ready(dc_ready),
        .mem_req(mem_req), .mem_we(mem_we), .mem_addr(mem_addr),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(mem_ready));
endmodule
