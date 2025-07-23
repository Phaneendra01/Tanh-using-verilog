This repository contains a Verilog implementation of the hyperbolic tangent (tanh) function using the CORDIC algorithm and GELU activation function implementation in Verilog, along with a comprehensive testbench, synthesis, and implementation results. This project aims to provide a clear and efficient hardware realization of the tanh function, crucial for various digital signal processing and neural network applications like GELU activation function implementation.

# Hyperbolic Tangent (tanh) Implementation in Verilog 🚀

## 1. Main Verilog Module (`tanh_cordic.v`) 💻

The core of this project is the `tanh_cordic.v` module, which implements the hyperbolic tangent function. It utilizes a pipelined CORDIC (Coordinate Rotation Digital Computer) algorithm for efficient computation. The module is parameterized to allow flexibility in data width, fractional width, and the number of CORDIC iterations.

```verilog
`timescale 1ns / 1ps

// Module: rhc_module

module rhc_module #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25,
    parameter ITERATIONS = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire signed [DATA_WIDTH-1:0] z_in,
    input  wire                  start,
    output reg  signed [DATA_WIDTH-1:0] exp_z_out,
    output reg                   done
);
    localparam TOTAL_STAGES = ITERATIONS + 2;
    localparam S_IDLE = 3'd0, S_REDUCE = 3'd1, S_CORDIC_RUN = 3'd2, S_CORDIC_WAIT = 3'd3, S_SCALE = 3'd4, S_DONE = 3'd5;
    reg [2:0] state, next_state;
    localparam signed [DATA_WIDTH-1:0] LN2 = 23265095;
    localparam signed [DATA_WIDTH-1:0] CORDIC_GAIN_INV = 40568448;
    reg signed [DATA_WIDTH-1:0] z_reduced;
    reg signed [5:0] k_factor;
    reg signed [DATA_WIDTH-1:0] cordic_result;
    reg [5:0] wait_counter;
    wire signed [DATA_WIDTH-1:0] x_pipe [0:TOTAL_STAGES], y_pipe [0:TOTAL_STAGES], z_pipe [0:TOTAL_STAGES];

    function [DATA_WIDTH-1:0] atanh_lut; input integer i; case (i) 1: atanh_lut = 18432512; 2: atanh_lut = 8569856; 3: atanh_lut = 4216320; 4: atanh_lut = 2099224; 5: atanh_lut = 1049600; 6: atanh_lut = 524288; 7: atanh_lut = 262144; 8: atanh_lut = 131072; 9: atanh_lut = 65536; 10: atanh_lut = 32768; 11: atanh_lut = 16384; 12: atanh_lut = 8192; 13: atanh_lut = 4096; 14: atanh_lut = 2048; 15: atanh_lut = 1024; 16: atanh_lut = 512; default: atanh_lut = 0; endcase endfunction
    function integer get_k; input integer i; if (i < 4) get_k = i + 1; else if (i < 13) get_k = i; else if (i == 13) get_k = 13; else get_k = i - 1; endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            z_reduced <= 32'd0;
            k_factor <= 6'd0;
            exp_z_out <= 32'd0;
            cordic_result <= 32'd0;
            wait_counter <= 6'd0;
        end else begin
            state <= next_state;
            if (state == S_IDLE && start) begin
                z_reduced <= z_in;
                k_factor <= 6'd0;
            end
            if (state == S_REDUCE) begin
                if (z_reduced > LN2) begin
                    z_reduced <= z_reduced - LN2;
                    k_factor <= k_factor + 1;
                end else if (z_reduced < -LN2) begin
                    z_reduced <= z_reduced + LN2;
                    k_factor <= k_factor - 1;
                end
            end
            if (state == S_CORDIC_RUN) begin
                wait_counter <= 6'd0;
            end else if (state == S_CORDIC_WAIT) begin
                wait_counter <= wait_counter + 1;
                if (wait_counter == TOTAL_STAGES - 1) begin
                    cordic_result <= x_pipe[TOTAL_STAGES] + y_pipe[TOTAL_STAGES];
                end
            end
            if (state == S_SCALE) begin
                if (k_factor > 0) begin
                    exp_z_out <= cordic_result << k_factor;
                end else begin
                    exp_z_out <= cordic_result >>> -k_factor;
                end
            end
        end
    end
    always @(*) begin
        next_state = state;
        done = 1'b0;
        case (state)
            S_IDLE: begin
                if (start) begin
                    next_state = S_REDUCE;
                end
            end
            S_REDUCE: begin
                if (z_reduced <= LN2 && z_reduced >= -LN2) begin
                    next_state = S_CORDIC_RUN;
                end
            end
            S_CORDIC_RUN: begin
                next_state = S_CORDIC_WAIT;
            end
            S_CORDIC_WAIT: begin
                if (wait_counter == TOTAL_STAGES - 1) begin
                    next_state = S_SCALE;
                end
            end
            S_SCALE: begin
                next_state = S_DONE;
            end
            S_DONE: begin
                done = 1'b1;
                next_state = S_IDLE;
            end
            default: begin
                next_state = S_IDLE;
            end
        endcase
    end
    assign x_pipe[0] = CORDIC_GAIN_INV;
    assign y_pipe[0] = 32'd0;
    assign z_pipe[0] = z_reduced;
    genvar i;
    generate for (i = 0; i < TOTAL_STAGES; i = i + 1) begin : cordic_pipeline_stage
        reg signed [DATA_WIDTH-1:0] x_reg, y_reg, z_reg;
        localparam integer K_SHIFT = get_k(i);
        wire d = (z_pipe[i] < 0);
        wire signed [DATA_WIDTH-1:0] x_shifted = x_pipe[i] >>> K_SHIFT;
        wire signed [DATA_WIDTH-1:0] y_shifted = y_pipe[i] >>> K_SHIFT;
        always @(posedge clk) begin
            if (d) begin
                x_reg <= x_pipe[i] - y_shifted;
                y_reg <= y_pipe[i] - x_shifted;
                z_reg <= z_pipe[i] + atanh_lut(K_SHIFT);
            end else begin
                x_reg <= x_pipe[i] + y_shifted;
                y_reg <= y_pipe[i] + x_shifted;
                z_reg <= z_pipe[i] - atanh_lut(K_SHIFT);
            end
        end
        assign x_pipe[i+1] = x_reg;
        assign y_pipe[i+1] = y_reg;
        assign z_pipe[i+1] = z_reg;
    end endgenerate
endmodule

// vlc_module

module vlc_module #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      start,
    input  wire signed [DATA_WIDTH-1:0] numerator_in,
    input  wire signed [DATA_WIDTH-1:0] denominator_in,
    output reg  signed [DATA_WIDTH-1:0] result_out,
    output reg                       done
);
    localparam NUM_ITERATIONS = DATA_WIDTH + FRAC_WIDTH;
    localparam S_IDLE   = 3'd0, S_SETUP  = 3'd1, S_CALC   = 3'd2, S_FINISH = 3'd3, S_DONE   = 3'd4;
    reg [2:0] state, next_state;

    reg [NUM_ITERATIONS + DATA_WIDTH : 0] p_reg;
    reg [DATA_WIDTH-1:0]                  divisor_reg;
    reg [6:0]                             iter_count;
    reg                                   result_sign;

    wire [NUM_ITERATIONS + DATA_WIDTH : 0] p_msbs_shifted;
    assign p_msbs_shifted = p_reg << 1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            result_out <= 0;
            done <= 1'b0;
            iter_count <= 0;
            p_reg <= 0;
            divisor_reg <= 0;
        end else begin
            state <= next_state;
            done  <= 1'b0;

            case (state)
                S_SETUP: begin
                    divisor_reg <= (denominator_in < 0) ? -denominator_in : denominator_in;
                    p_reg <= { {(DATA_WIDTH+1){1'b0}}, {(numerator_in < 0) ? -numerator_in : numerator_in} } << FRAC_WIDTH;
                    iter_count <= NUM_ITERATIONS;
                    result_sign <= numerator_in[DATA_WIDTH-1] ^ denominator_in[DATA_WIDTH-1];
                end

                S_CALC: begin
                    if (iter_count > 0) begin
                        if (p_msbs_shifted[NUM_ITERATIONS+DATA_WIDTH : NUM_ITERATIONS] >= divisor_reg) begin
                            [NUM_ITERATIONS-1:1] to ensure correct width
                            p_reg <= { (p_msbs_shifted[NUM_ITERATIONS+DATA_WIDTH : NUM_ITERATIONS] - divisor_reg),
                                       p_msbs_shifted[NUM_ITERATIONS-1:1],
                                       1'b1 };
                        end else begin
                            p_reg <= p_msbs_shifted;
                        end
                        iter_count <= iter_count - 1;
                    end
                end

                S_FINISH: begin
                    if (result_sign) begin
                        result_out <= -p_reg[DATA_WIDTH-1:0];
                    end else begin
                        result_out <= p_reg[DATA_WIDTH-1:0];
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end

    always @(*) begin
        next_state = state;
        case(state)
            S_IDLE:   if (start) next_state = S_SETUP;
            S_SETUP:  if (denominator_in == 0) next_state = S_FINISH; else next_state = S_CALC;
            S_CALC:   if (iter_count == 0) next_state = S_FINISH;
            S_FINISH: next_state = S_DONE;
            S_DONE:   next_state = S_IDLE;
            default:  next_state = S_IDLE;
        endcase
    end
endmodule

// Module: tanh_cordic (Top-Level)

module tanh_cordic #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25,
    parameter ITERATIONS = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire signed [DATA_WIDTH-1:0] x_in,
    input  wire                  start,
    output reg  signed [DATA_WIDTH-1:0] tanh_out,
    output reg                   done
);
    localparam S_IDLE = 4'd0, S_COMPUTE_EXP = 4'd1, S_WAIT_EXP = 4'd2, S_COMPUTE_NEXP = 4'd3, S_WAIT_NEXP = 4'd4, S_START_DIVIDE = 4'd5, S_WAIT_DIVIDE = 4'd6, S_DONE = 4'd7;
    reg [3:0] state, next_state;
    reg exp_start, div_start;
    reg signed [DATA_WIDTH-1:0] exp_input;
    wire signed [DATA_WIDTH-1:0] exp_output, div_result;
    wire exp_done, div_done;
    reg signed [DATA_WIDTH-1:0] exp_pos, exp_neg;
    localparam signed [DATA_WIDTH-1:0] ONE = 33554432;

    rhc_module #(.DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .ITERATIONS(ITERATIONS)) exp_inst (.clk(clk), .rst_n(rst_n), .z_in(exp_input), .start(exp_start), .exp_z_out(exp_output), .done(exp_done));
    vlc_module #(.DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH)) div_inst (.clk(clk), .rst_n(rst_n), .start(div_start), .numerator_in(exp_pos - exp_neg), .denominator_in(exp_pos + exp_neg), .result_out(div_result), .done(div_done));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            tanh_out <= 32'd0;
            exp_pos <= 32'd0;
            exp_neg <= 32'd0;
            exp_start <= 1'b0;
            exp_input <= 32'd0;
            div_start <= 1'b0;
        end else begin
            state <= next_state;
            div_start <= 1'b0;
            exp_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        exp_start <= 1'b1;
                        exp_input <= x_in;
                    end
                end
                S_WAIT_EXP: begin
                    if (exp_done) begin
                        exp_pos <= exp_output;
                        exp_start <= 1'b1;
                        exp_input <= -x_in;
                    end
                end
                S_WAIT_NEXP: begin
                    if (exp_done) begin
                        exp_neg <= exp_output;
                    end
                end
                S_START_DIVIDE: begin
                    div_start <= 1'b1;
                end
                S_WAIT_DIVIDE: begin
                    if (div_done) begin
                        tanh_out <= div_result;
                    end
                end
                S_DONE: begin
                    if (tanh_out > ONE) begin
                        tanh_out <= ONE;
                    end else if (tanh_out < -ONE) begin
                        tanh_out <= -ONE;
                    end
                end
            endcase
        end
    end
    always @(*) begin
        next_state = state;
        done = 1'b0;
        case (state)
            S_IDLE:         if (start) next_state = S_COMPUTE_EXP;
            S_COMPUTE_EXP:  next_state = S_WAIT_EXP;
            S_WAIT_EXP:     if (exp_done) next_state = S_COMPUTE_NEXP;
            S_COMPUTE_NEXP: next_state = S_WAIT_NEXP;
            S_WAIT_NEXP:    if (exp_done) next_state = S_START_DIVIDE;
            S_START_DIVIDE: next_state = S_WAIT_DIVIDE;
            S_WAIT_DIVIDE:  if (div_done) next_state = S_DONE;
            S_DONE:         begin done = 1'b1; next_state = S_IDLE; end
            default:        next_state = S_IDLE;
        endcase
    end
endmodule
```

## 2. Testbench (`tb_tanh_cordic.v`) 🧪

A thorough testbench (`tb_tanh_cordic.v`) is provided to verify the functionality and accuracy of the `tanh_cordic` module. It stimulates the design with a range of input values and compares the computed tanh output against expected values, demonstrating the precision of the CORDIC implementation.

```verilog
module tb_tanh_cordic;
    parameter DATA_WIDTH = 32, FRAC_WIDTH = 25, ITERATIONS = 24, CLK_PERIOD = 10;
    reg clk, rst_n, start;
    reg signed [DATA_WIDTH-1:0] x_in;
    wire signed [DATA_WIDTH-1:0] tanh_out;
    wire done;
    reg signed [DATA_WIDTH-1:0] test_values [0:12];
    real test_inputs [0:12];
    integer i, j;

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    initial begin
        for (j = -6; j <= 6; j = j + 1) begin
            i = j + 6;
            test_inputs[i] = j;
            test_values[i] = j * (1 << FRAC_WIDTH);
        end
    end

    tanh_cordic #(.DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .ITERATIONS(ITERATIONS)) dut (.clk(clk), .rst_n(rst_n), .x_in(x_in), .start(start), .tanh_out(tanh_out), .done(done));

    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        x_in  = 32'd0;
        #(CLK_PERIOD * 2);
        rst_n = 1'b1;
        #(CLK_PERIOD * 2);
        $display("=== TANH CORDIC Test Results (-6 to +6) ===");
        $display("Input\t\tOutput (Fixed)\tOutput (Real)\tExpected");
        $display("-----\t\t--------------\t-------------\t--------");
        for (i = 0; i < 13; i = i + 1) begin
            x_in = test_values[i];
            start = 1'b1;
            #CLK_PERIOD;
            start = 1'b0;
            wait(done);
            #CLK_PERIOD;
            $display("%f\t%d\t%f\t%f", test_inputs[i], tanh_out, $signed(tanh_out) / (2.0**FRAC_WIDTH), $tanh(test_inputs[i]));
            #(CLK_PERIOD * 5);
        end
        $display("\n=== Test Complete ===");
        $finish;
    end
    initial begin
        #(CLK_PERIOD * 30000);
        $display("ERROR: Test timeout!");
        $finish;
    end
endmodule
```

## 3. Simulation Outputs (Obtained vs. Expected) 📊

Below are the simulation results, showcasing the outputs obtained from the `tanh_cordic` module compared to the ideal hyperbolic tangent values. This section validates the correctness of the Verilog implementation.

<img width="510" height="396" alt="Image" src="https://github.com/user-attachments/assets/b35b040d-be75-4452-986a-0b171337f719" />

## 4. Synthesis Results and Synthesized Design ⚙️

This section presents the synthesis report and a visual representation of the synthesized design. It provides insights into the hardware resources utilized and the structural view of the implemented logic.

### Synthesis Report Summary

<img width="346" height="368" alt="Image" src="https://github.com/user-attachments/assets/fd7a9554-b4f3-4a03-9543-808fdd7050a2" />

<img width="1065" height="473" alt="Image" src="https://github.com/user-attachments/assets/3a88a1a4-d3ff-4d1f-999d-e73f226cbb01" />

<img width="1423" height="386" alt="Image" src="https://github.com/user-attachments/assets/fbf91fac-26c5-4bf4-ac87-79367e3454af" />

<img width="1350" height="598" alt="Image" src="https://github.com/user-attachments/assets/c3d7b67b-aff9-4efd-884d-234ee2d6b0c3" />

<img width="1080" height="390" alt="Image" src="https://github.com/user-attachments/assets/ee0f3854-fb8b-4e93-9755-021cda2dba82" />

<img width="1920" height="1020" alt="Image" src="https://github.com/user-attachments/assets/9b16d725-8c65-45c6-b054-1df958309ef5" />

## 5. Implementation Results and Implemented Design 🛠️

Following synthesis, the design undergoes place and route. This section details the implementation results, including timing, power, and area reports, along with a visual of the final implemented design on the target FPGA.

### Implementation Report Summary

_Details about implementation results, e.g., timing, power, area, etc._

<img width="392" height="368" alt="Image" src="https://github.com/user-attachments/assets/78126f03-9741-4a43-9ff0-a3221435db12" />

<img width="1479" height="359" alt="Image" src="https://github.com/user-attachments/assets/3bd67e17-838e-40af-b6dd-25ca698a9426" />

<img width="1141" height="577" alt="Image" src="https://github.com/user-attachments/assets/41b9a12c-66b6-4e2d-b1f2-ca06bb175e91" />

<img width="1228" height="335" alt="Image" src="https://github.com/user-attachments/assets/46b4e595-0eda-42dc-92b6-677f9ea7691e" />

<img width="1920" height="1020" alt="Image" src="https://github.com/user-attachments/assets/27d9b21d-6654-44a5-b5b4-0ec6053eb23c" />

## 6. Timing Constraint ⏰

The following timing constraint was applied during synthesis and implementation to ensure the design meets the required performance specifications.

```tcl
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 3.000} -add [get_ports clk]
```

## 7. Timing Report Generated After Implementation ⏱️

The post-implementation timing report confirms that all timing requirements are met, ensuring the reliable operation of the `tanh_cordic` module at the specified clock frequency.

<img width="1479" height="359" alt="Image" src="https://github.com/user-attachments/assets/3bd67e17-838e-40af-b6dd-25ca698a9426" />

## Getting Started ✨

To replicate these results or use the module in your own project, follow these steps:

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/tanh-cordic-verilog.git
    cd tanh-cordic-verilog
    ```
2.  **Simulate:** Use your preferred Verilog simulator (eg. Vivado which was used for this project) to run `tb_tanh_cordic.v`.
3.  **Synthesize and Implement:** Use an FPGA design suite (e.g., Xilinx Vivado, Intel Quartus) to synthesize and implement `tanh_cordic.v` for your target device.

# GELU Activation Function Implementation in Verilog 🚀

## 1. Main Verilog Module (`gelu.v`) 💻

The core of this project is the `gelu.v` module, which implements the GELU activation function. It utilizes a pipelined CORDIC (Coordinate Rotation Digital Computer) algorithm for efficient computation. The module is parameterized to allow flexibility in data width, fractional width, and the number of CORDIC iterations.

```verilog
`timescale 1ns / 1ps

// ============================================================================
// Module: rhc_module (Original from User Prompt)
// Computes e^z using a pipelined hyperbolic CORDIC algorithm.
// ============================================================================
module rhc_module #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25,
    parameter ITERATIONS = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire signed [DATA_WIDTH-1:0] z_in,
    input  wire                  start,
    output reg  signed [DATA_WIDTH-1:0] exp_z_out,
    output reg                   done
);
    localparam TOTAL_STAGES = ITERATIONS + 2;
    localparam S_IDLE = 3'd0, S_REDUCE = 3'd1, S_CORDIC_RUN = 3'd2, S_CORDIC_WAIT = 3'd3, S_SCALE = 3'd4, S_DONE = 3'd5;
    reg [2:0] state, next_state;
    localparam signed [DATA_WIDTH-1:0] LN2 = 23265095;
    localparam signed [DATA_WIDTH-1:0] CORDIC_GAIN_INV = 40568448;
    reg signed [DATA_WIDTH-1:0] z_reduced;
    reg signed [5:0] k_factor;
    reg signed [DATA_WIDTH-1:0] cordic_result;
    reg [5:0] wait_counter;
    wire signed [DATA_WIDTH-1:0] x_pipe [0:TOTAL_STAGES], y_pipe [0:TOTAL_STAGES], z_pipe [0:TOTAL_STAGES];

    function [DATA_WIDTH-1:0] atanh_lut; input integer i; case (i) 1: atanh_lut = 18432512; 2: atanh_lut = 8569856; 3: atanh_lut = 4216320; 4: atanh_lut = 2099224; 5: atanh_lut = 1049600; 6: atanh_lut = 524288; 7: atanh_lut = 262144; 8: atanh_lut = 131072; 9: atanh_lut = 65536; 10: atanh_lut = 32768; 11: atanh_lut = 16384; 12: atanh_lut = 8192; 13: atanh_lut = 4096; 14: atanh_lut = 2048; 15: atanh_lut = 1024; 16: atanh_lut = 512; default: atanh_lut = 0; endcase endfunction
    function integer get_k; input integer i; if (i < 4) get_k = i + 1; else if (i < 13) get_k = i; else if (i == 13) get_k = 13; else get_k = i - 1; endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            z_reduced <= 32'd0;
            k_factor <= 6'd0;
            exp_z_out <= 32'd0;
            cordic_result <= 32'd0;
            wait_counter <= 6'd0;
        end else begin
            state <= next_state;
            if (state == S_IDLE && start) begin
                z_reduced <= z_in;
                k_factor <= 6'd0;
            end
            if (state == S_REDUCE) begin
                if (z_reduced > LN2) begin
                    z_reduced <= z_reduced - LN2;
                    k_factor <= k_factor + 1;
                end else if (z_reduced < -LN2) begin
                    z_reduced <= z_reduced + LN2;
                    k_factor <= k_factor - 1;
                end
            end
            if (state == S_CORDIC_RUN) begin
                wait_counter <= 6'd0;
            end else if (state == S_CORDIC_WAIT) begin
                wait_counter <= wait_counter + 1;
                if (wait_counter == TOTAL_STAGES - 1) begin
                    cordic_result <= x_pipe[TOTAL_STAGES] + y_pipe[TOTAL_STAGES];
                end
            end
            if (state == S_SCALE) begin
                if (k_factor > 0) begin
                    exp_z_out <= cordic_result << k_factor;
                end else begin
                    exp_z_out <= cordic_result >>> -k_factor;
                end
            end
        end
    end
    always @(*) begin
        next_state = state;
        done = 1'b0;
        case (state)
            S_IDLE: begin
                if (start) begin
                    next_state = S_REDUCE;
                end
            end
            S_REDUCE: begin
                if (z_reduced <= LN2 && z_reduced >= -LN2) begin
                    next_state = S_CORDIC_RUN;
                end
            end
            S_CORDIC_RUN: begin
                next_state = S_CORDIC_WAIT;
            end
            S_CORDIC_WAIT: begin
                if (wait_counter == TOTAL_STAGES - 1) begin
                    next_state = S_SCALE;
                end
            end
            S_SCALE: begin
                next_state = S_DONE;
            end
            S_DONE: begin
                done = 1'b1;
                next_state = S_IDLE;
            end
            default: begin
                next_state = S_IDLE;
            end
        endcase
    end
    assign x_pipe[0] = CORDIC_GAIN_INV;
    assign y_pipe[0] = CORDIC_GAIN_INV;
    assign z_pipe[0] = z_reduced;
    genvar i;
    generate for (i = 0; i < TOTAL_STAGES; i = i + 1) begin : cordic_pipeline_stage
        reg signed [DATA_WIDTH-1:0] x_reg, y_reg, z_reg;
        localparam integer K_SHIFT = get_k(i);
        wire d = (z_pipe[i] < 0);
        wire signed [DATA_WIDTH-1:0] x_shifted = x_pipe[i] >>> K_SHIFT;
        wire signed [DATA_WIDTH-1:0] y_shifted = y_pipe[i] >>> K_SHIFT;
        always @(posedge clk) begin
            if (d) begin
                x_reg <= x_pipe[i] - y_shifted;
                y_reg <= y_pipe[i] - x_shifted;
                z_reg <= z_pipe[i] + atanh_lut(K_SHIFT);
            end else begin
                x_reg <= x_pipe[i] + y_shifted;
                y_reg <= y_pipe[i] + x_shifted;
                z_reg <= z_pipe[i] - atanh_lut(K_SHIFT);
            end
        end
        assign x_pipe[i+1] = x_reg;
        assign y_pipe[i+1] = y_reg;
        assign z_pipe[i+1] = z_reg;
    end endgenerate
endmodule


// ============================================================================
// Module: vlc_module (Original from User Prompt)
// Fixed-point divider using a non-restoring algorithm.
// ============================================================================
module vlc_module #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire signed [DATA_WIDTH-1:0] numerator_in,
    input  wire signed [DATA_WIDTH-1:0] denominator_in,
    output reg  signed [DATA_WIDTH-1:0] result_out,
    output reg                        done
);
    localparam NUM_ITERATIONS = DATA_WIDTH + FRAC_WIDTH;
    localparam S_IDLE   = 3'd0, S_SETUP  = 3'd1, S_CALC   = 3'd2, S_FINISH = 3'd3, S_DONE   = 3'd4;
    reg [2:0] state, next_state;

    reg [NUM_ITERATIONS + DATA_WIDTH : 0] p_reg;
    reg [DATA_WIDTH-1:0]                  divisor_reg;
    reg [6:0]                             iter_count;
    reg                                   result_sign;
    
    wire [NUM_ITERATIONS + DATA_WIDTH : 0] p_msbs_shifted;
    assign p_msbs_shifted = p_reg << 1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            result_out <= 0;
            done <= 1'b0;
            iter_count <= 0;
            p_reg <= 0;
            divisor_reg <= 0;
        end else begin
            state <= next_state;
            done  <= 1'b0;

            case (state)
                S_SETUP: begin
                    divisor_reg <= (denominator_in < 0) ? -denominator_in : denominator_in;
                    p_reg <= { {(DATA_WIDTH+1){1'b0}}, {(numerator_in < 0) ? -numerator_in : numerator_in} } << FRAC_WIDTH;
                    iter_count <= NUM_ITERATIONS;
                    result_sign <= numerator_in[DATA_WIDTH-1] ^ denominator_in[DATA_WIDTH-1];
                end

                S_CALC: begin
                    if (iter_count > 0) begin
                        if (p_msbs_shifted[NUM_ITERATIONS+DATA_WIDTH : NUM_ITERATIONS] >= divisor_reg) begin
                            p_reg <= { (p_msbs_shifted[NUM_ITERATIONS+DATA_WIDTH : NUM_ITERATIONS] - divisor_reg), 
                                       p_msbs_shifted[NUM_ITERATIONS-1:1], 
                                       1'b1 };
                        end else begin
                            p_reg <= p_msbs_shifted;
                        end
                        iter_count <= iter_count - 1;
                    end
                end
                
                S_FINISH: begin
                    if (result_sign) begin
                        result_out <= -p_reg[DATA_WIDTH-1:0];
                    end else begin
                        result_out <= p_reg[DATA_WIDTH-1:0];
                    end
                end
                
                S_DONE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end

    always @(*) begin
        next_state = state;
        case(state)
            S_IDLE:   if (start) next_state = S_SETUP;
            S_SETUP:  if (denominator_in == 0) next_state = S_FINISH; else next_state = S_CALC;
            S_CALC:   if (iter_count == 0) next_state = S_FINISH;
            S_FINISH: next_state = S_DONE;
            S_DONE:   next_state = S_IDLE;
            default:  next_state = S_IDLE;
        endcase
    end
endmodule


// ============================================================================
// Module: tanh_cordic (Original from User Prompt)
// Computes tanh(x) using the e^z and division modules.
// ============================================================================
module tanh_cordic #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25,
    parameter ITERATIONS = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire signed [DATA_WIDTH-1:0] x_in,
    input  wire                  start,
    output reg  signed [DATA_WIDTH-1:0] tanh_out,
    output reg                   done
);
    localparam S_IDLE = 4'd0, S_COMPUTE_EXP = 4'd1, S_WAIT_EXP = 4'd2, S_COMPUTE_NEXP = 4'd3, S_WAIT_NEXP = 4'd4, S_START_DIVIDE = 4'd5, S_WAIT_DIVIDE = 4'd6, S_DONE = 4'd7;
    reg [3:0] state, next_state;
    reg exp_start, div_start;
    reg signed [DATA_WIDTH-1:0] exp_input;
    wire signed [DATA_WIDTH-1:0] exp_output, div_result;
    wire exp_done, div_done;
    reg signed [DATA_WIDTH-1:0] exp_pos, exp_neg;
    localparam signed [DATA_WIDTH-1:0] ONE = 33554432;

    rhc_module #(.DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .ITERATIONS(ITERATIONS)) exp_inst (.clk(clk), .rst_n(rst_n), .z_in(exp_input), .start(exp_start), .exp_z_out(exp_output), .done(exp_done));
    vlc_module #(.DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH)) div_inst (.clk(clk), .rst_n(rst_n), .start(div_start), .numerator_in(exp_pos - exp_neg), .denominator_in(exp_pos + exp_neg), .result_out(div_result), .done(div_done));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            tanh_out <= 32'd0;
            exp_pos <= 32'd0;
            exp_neg <= 32'd0;
            exp_start <= 1'b0;
            exp_input <= 32'd0;
            div_start <= 1'b0;
        end else begin
            state <= next_state;
            div_start <= 1'b0;
            exp_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        exp_start <= 1'b1;
                        exp_input <= x_in;
                    end
                end
                S_WAIT_EXP: begin
                    if (exp_done) begin
                        exp_pos <= exp_output;
                        exp_start <= 1'b1;
                        exp_input <= -x_in;
                    end
                end
                S_WAIT_NEXP: begin
                    if (exp_done) begin
                        exp_neg <= exp_output;
                    end
                end
                S_START_DIVIDE: begin
                    div_start <= 1'b1;
                end
                S_WAIT_DIVIDE: begin
                    if (div_done) begin
                        tanh_out <= div_result;
                    end
                end
                S_DONE: begin
                    if (tanh_out > ONE) begin
                        tanh_out <= ONE;
                    end else if (tanh_out < -ONE) begin
                        tanh_out <= -ONE;
                    end
                end
            endcase
        end
    end
    always @(*) begin
        next_state = state;
        done = 1'b0;
        case (state)
            S_IDLE:         if (start) next_state = S_COMPUTE_EXP;
            S_COMPUTE_EXP:  next_state = S_WAIT_EXP;
            S_WAIT_EXP:     if (exp_done) next_state = S_COMPUTE_NEXP;
            S_COMPUTE_NEXP: next_state = S_WAIT_NEXP;
            S_WAIT_NEXP:    if (exp_done) next_state = S_START_DIVIDE;
            S_START_DIVIDE: next_state = S_WAIT_DIVIDE;
            S_WAIT_DIVIDE:  if (div_done) next_state = S_DONE;
            S_DONE:         begin done = 1'b1; next_state = S_IDLE; end
            default:        next_state = S_IDLE;
        endcase
    end
endmodule


// ============================================================================
// Module: gelu_module (Top-Level for GELU)
// Computes GELU(x) using the tanh approximation.
// GELU(x) approx 0.5x * (1 + tanh(sqrt(2/pi)*(x + 0.044715x^3)))
// ============================================================================
module gelu_module #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 25,
    parameter ITERATIONS = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire signed [DATA_WIDTH-1:0] x_in,
    input  wire                  start,
    output reg  signed [DATA_WIDTH-1:0] gelu_out,
    output reg                   done
);
    // State machine definition
    localparam S_IDLE = 0, S_CALC_X2 = 1, S_CALC_X3 = 2, S_CALC_TERM1 = 3,
               S_ADD1 = 4, S_CALC_TANH_ARG = 5, S_TANH_START = 6, S_TANH_WAIT = 7,
               S_ADD2 = 8, S_FINAL_PROD = 9, S_FINAL_SHIFT = 10, S_DONE = 11;
    reg [3:0] state, next_state;

    // Fixed-point constants for the GELU formula (Q7.25 format)
    localparam signed [DATA_WIDTH-1:0] C1 = 1500000;    // 0.044715 * 2^25
    localparam signed [DATA_WIDTH-1:0] C2 = 26765061;   // sqrt(2/pi) * 2^25
    localparam signed [DATA_WIDTH-1:0] ONE = 33554432;  // 1.0 * 2^25

    // Internal registers for intermediate calculations
    reg signed [DATA_WIDTH-1:0] x, x2, x3, term1, term2, tanh_arg, tanh_res, final_prod;
    
    // Wires and control signals for the tanh module
    wire signed [DATA_WIDTH-1:0] tanh_output;
    wire tanh_done;
    reg tanh_start;

    // Instantiate the tanh_cordic module
    tanh_cordic #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_WIDTH(FRAC_WIDTH),
        .ITERATIONS(ITERATIONS)
    ) tanh_inst (
        .clk(clk),
        .rst_n(rst_n),
        .x_in(tanh_arg),
        .start(tanh_start),
        .tanh_out(tanh_output),
        .done(tanh_done)
    );

    // This function performs fixed-point multiplication: (a * b) >> FRAC_WIDTH
    // It uses a temporary 64-bit register to avoid overflow.
    function signed [DATA_WIDTH-1:0] mult_fixed;
        input signed [DATA_WIDTH-1:0] a;
        input signed [DATA_WIDTH-1:0] b;
        reg signed [2*DATA_WIDTH-1:0] temp_prod;
        begin
            temp_prod = a * b;
            mult_fixed = temp_prod >>> FRAC_WIDTH;
        end
    endfunction

    // Sequential logic for the GELU state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            gelu_out <= 0;
            tanh_start <= 1'b0;
            x <= 0; x2 <= 0; x3 <= 0; term1 <= 0; term2 <= 0;
            tanh_arg <= 0; tanh_res <= 0; final_prod <= 0;
        end else begin
            state <= next_state;
            tanh_start <= 1'b0; // Default to not starting

            case (state)
                S_IDLE: if (start) begin
                    x <= x_in;
                end
                S_CALC_X2: x2 <= mult_fixed(x, x);
                S_CALC_X3: x3 <= mult_fixed(x2, x);
                S_CALC_TERM1: term1 <= mult_fixed(x3, C1);
                S_ADD1: term2 <= x + term1;
                S_CALC_TANH_ARG: tanh_arg <= mult_fixed(term2, C2);
                S_TANH_START: tanh_start <= 1'b1;
                S_TANH_WAIT: if (tanh_done) begin
                    tanh_res <= tanh_output;
                end
                S_ADD2: term1 <= ONE + tanh_res;
                S_FINAL_PROD: final_prod <= mult_fixed(term1, x);
                S_FINAL_SHIFT: gelu_out <= final_prod >>> 1; // Multiply by 0.5
            endcase
        end
    end

    // Combinational logic for state transitions
    always @(*) begin
        next_state = state;
        done = 1'b0;
        case (state)
            S_IDLE:          if (start) next_state = S_CALC_X2;
            S_CALC_X2:       next_state = S_CALC_X3;
            S_CALC_X3:       next_state = S_CALC_TERM1;
            S_CALC_TERM1:    next_state = S_ADD1;
            S_ADD1:          next_state = S_CALC_TANH_ARG;
            S_CALC_TANH_ARG: next_state = S_TANH_START;
            S_TANH_START:    next_state = S_TANH_WAIT;
            S_TANH_WAIT:     if (tanh_done) next_state = S_ADD2;
            S_ADD2:          next_state = S_FINAL_PROD;
            S_FINAL_PROD:    next_state = S_FINAL_SHIFT;
            S_FINAL_SHIFT:   next_state = S_DONE;
            S_DONE:          begin done = 1'b1; next_state = S_IDLE; end
            default:         next_state = S_IDLE;
        endcase
    end
endmodule

```

## 2. Testbench (`tb_gelu.v`) 🧪

A thorough testbench (`tb_gelu.v`) is provided to verify the functionality and accuracy of the `gelu` module. It stimulates the design with a range of input values and compares the computed tanh output against expected values, demonstrating the precision of the CORDIC implementation.

```verilog
`timescale 1ns / 1ps

module tb_gelu_cordic;

    // Parameters for the test
    parameter DATA_WIDTH = 32;
    parameter FRAC_WIDTH = 25;
    parameter ITERATIONS = 24;
    parameter CLK_PERIOD = 10;
    // UPDATED: Number of tests for the new range [-3, 3]
    parameter NUM_TESTS  = 13; 

    // Testbench signals
    reg clk;
    reg rst_n;
    reg start;
    reg signed [DATA_WIDTH-1:0] x_in;
    wire signed [DATA_WIDTH-1:0] gelu_out;
    wire done;

    // Test vectors and loop variables
    reg signed [DATA_WIDTH-1:0] test_values [0:NUM_TESTS-1];
    real test_inputs [0:NUM_TESTS-1];
    integer i, j;
    
    // Real variables for calculating the expected golden value in the testbench
    real x_real, tanh_arg_real, tanh_res_real, expected_gelu;

    // Clock generation
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // Initialize test vectors
    initial begin
        for (j = 0; j < NUM_TESTS; j = j + 1) begin
            // UPDATED: Start range from -3.0
            test_inputs[j] = -3.0 + j * 0.5;
            test_values[j] = test_inputs[j] * (1 << FRAC_WIDTH);
        end
    end

    // Instantiate the design under test (DUT)
    gelu_module #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_WIDTH(FRAC_WIDTH),
        .ITERATIONS(ITERATIONS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .x_in(x_in),
        .start(start),
        .gelu_out(gelu_out),
        .done(done)
    );

    // Main test sequence
    initial begin
        // Reset and initialize
        rst_n = 1'b0;
        start = 1'b0;
        x_in  = 32'd0;
        #(CLK_PERIOD * 2);
        rst_n = 1'b1;
        #(CLK_PERIOD * 2);

        // Print header for results
        // UPDATED: Display string for the new range
        $display("=== GELU CORDIC Test Results (-3.0 to +3.0) ===");
        $display("Input\t\tOutput (Fixed)\tOutput (Real)\tExpected (Real)");
        $display("-----\t\t--------------\t-------------\t---------------");

        // Loop through all test values
        for (i = 0; i < NUM_TESTS; i = i + 1) begin
            x_in = test_values[i];
            start = 1'b1;
            #CLK_PERIOD;
            start = 1'b0;
            
            // Wait for the calculation to complete
            wait(done);
            #CLK_PERIOD;

            // Calculate expected value for comparison
            x_real = test_inputs[i];
            tanh_arg_real = $sqrt(2.0/3.1415926535) * (x_real + 0.044715 * x_real * x_real * x_real);
            tanh_res_real = $tanh(tanh_arg_real);
            expected_gelu = 0.5 * x_real * (1.0 + tanh_res_real);

            // Display results
            $display("%f\t%d\t%f\t%f", 
                test_inputs[i], 
                gelu_out, 
                $signed(gelu_out) / (2.0**FRAC_WIDTH), 
                expected_gelu);
            
            #(CLK_PERIOD * 5);
        end

        $display("\n=== Test Complete ===");
        $finish;
    end

    // Test timeout
    initial begin
        #(CLK_PERIOD * 50000);
        $display("ERROR: Test timeout!");
        $finish;
    end

endmodule
```

## 3. Simulation Outputs (Obtained vs. Expected) 📊

Below are the simulation results, showcasing the outputs obtained from the `gelu` module compared to the ideal gelu activation function values. This section validates the correctness of the Verilog implementation.

<img width="546" height="403" alt="Image" src="https://github.com/user-attachments/assets/02012c1b-1456-4a0b-b94b-739e9d729abc" />

## 4. Synthesis Results and Synthesized Design ⚙️

This section presents the synthesis report and a visual representation of the synthesized design. It provides insights into the hardware resources utilized and the structural view of the implemented logic.

### Synthesis Report Summary

<img width="346" height="368" alt="Image" src="https://github.com/user-attachments/assets/fff67c82-fe29-4f00-84d7-13a4eb24a49c" />

<img width="1065" height="473" alt="Image" src="https://github.com/user-attachments/assets/47ac45bc-791c-4d31-be8e-c8513ca91250" />

<img width="1413" height="354" alt="Image" src="https://github.com/user-attachments/assets/aba5b52b-0bf9-49e7-8060-7999b75bf10f" />

<img width="1162" height="596" alt="Image" src="https://github.com/user-attachments/assets/8766763a-3f79-45ef-aca5-ea8f5131bc88" />

<img width="1093" height="418" alt="Image" src="https://github.com/user-attachments/assets/f893b96f-0e12-4c06-841a-947db6898819" />

<img width="1920" height="1020" alt="Image" src="https://github.com/user-attachments/assets/f3b3350f-a440-4a5b-9076-a6e66885be09" />
