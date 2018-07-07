module ml505top (
   input  [4:0]  GPIO_COMPSW,
   output [7:0]  GPIO_LED,
   input        USER_CLK,

   input VGA_IN_DATA_CLK,
	input VGA_IN_HSOUT,
	input VGA_IN_VSOUT,
	input [7:0] VGA_IN_BLUE,
	input [7:0] VGA_IN_GREEN,
	input [7:0] VGA_IN_RED,

   output [11:0] DVI_D,
	output        DVI_DE,
	output        DVI_H,
	output        DVI_RESET_B,
	output        DVI_V,
	output        DVI_XCLK_N,
	output        DVI_XCLK_P,

	inout         IIC_SCL_VIDEO,
	inout         IIC_SDA_VIDEO
);

  wire rst;

	// Clocks
	wire user_clk_g;

  wire cpu_clk;
  wire cpu_clk_g;

  wire clk0;
  wire clk0_g;

  wire clk90;
  wire clk90_g;

  wire clkdiv0;
  wire clkdiv0_g;

  wire clk200;
  wire clk200_g;

  wire pll_lock;

  wire clk50;
  wire clk50_g;

  PLL_BASE
  #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKFBOUT_MULT(24),
    .CLKFBOUT_PHASE(0.0),
    .CLKIN_PERIOD(10.0),

    .CLKOUT0_DIVIDE(24),
    .CLKOUT0_DUTY_CYCLE(0.5),
    .CLKOUT0_PHASE(0.0),

    .CLKOUT1_DIVIDE(3),
    .CLKOUT1_DUTY_CYCLE(0.5),
    .CLKOUT1_PHASE(0.0),

    .CLKOUT2_DIVIDE(3),
    .CLKOUT2_DUTY_CYCLE(0.5),
    .CLKOUT2_PHASE(0.0),

    .CLKOUT3_DIVIDE(3),
    .CLKOUT3_DUTY_CYCLE(0.5),
    .CLKOUT3_PHASE(90.0),

    .CLKOUT4_DIVIDE(6),
    .CLKOUT4_DUTY_CYCLE(0.5),
    .CLKOUT4_PHASE(0.0),

    .CLKOUT5_DIVIDE(12),
    .CLKOUT5_DUTY_CYCLE(0.5),
    .CLKOUT5_PHASE(0.0),

    .COMPENSATION("SYSTEM_SYNCHRONOUS"),
    .DIVCLK_DIVIDE(4),
    .REF_JITTER(0.100)
  )
  user_clk_pll
  (
    .CLKFBOUT(pll_fb),
    .CLKOUT0(cpu_clk),
    .CLKOUT1(clk200),
    .CLKOUT2(clk0),
    .CLKOUT3(clk90),
    .CLKOUT4(clkdiv0),
    .CLKOUT5(clk50),
    .LOCKED(pll_lock),
    .CLKFBIN(pll_fb),
    .CLKIN(user_clk_g),
    .RST(1'b0)
  );

  IBUFG user_clk_buf ( .I(USER_CLK), .O(user_clk_g) );
  BUFG  cpu_clk_buf  ( .I(cpu_clk),  .O(cpu_clk_g)  );
  BUFG  clk200_buf   ( .I(clk200),   .O(clk200_g)   );
  BUFG  clk0_buf     ( .I(clk0),     .O(clk0_g)     );
  BUFG  clkdiv50_buf ( .I(clk50),    .O(clk50_g)    );
  BUFG  clk90_buf    ( .I(clk90),    .O(clk90_g)    );
  BUFG  clkdiv0_buf  ( .I(clkdiv0),  .O(clkdiv0_g)  );

	Debouncer rst_parse(
      .clk(cpu_clk_g),
      .in(GPIO_COMPSW[4] | GPIO_COMPSW[3] | GPIO_COMPSW[2] | GPIO_COMPSW[1] | GPIO_COMPSW[0]),
      .out(rst));

wire[7:0] inp_wire1;
reg [7:0] inp_reg1;
wire [7:0] out_wire1;
wire rdy_wire1;
wire[7:0] inp_wire2;
reg [7:0] inp_reg2;
wire [7:0] out_wire2;
wire rdy_wire2;
wire[7:0] inp_wire3;
reg [7:0] inp_reg3;
wire [7:0] out_wire3;
wire rdy_wire3;
wire rstconv;
reg rst_conv;
//assign rst_conv =1;
assign rstconv =rst_conv;
assign inp_wire1 =inp_reg1;
assign inp_wire2 =inp_reg2;
assign inp_wire3 =inp_reg3;
//reg ready;
wire rdy_wire;
//wire rdy_wire1;
//wire rdy_wire2;
//wire rdy_wire3;
//reg r1;
//reg r2;
//reg r3;
//assign 
assign rdy_wire = rdy_wire1 & rdy_wire2 & rdy_wire3;

//assign rdy_wire = ready;
top conv1(.datain(inp_wire1),
 .dataout(out_wire1),
 .clock(cpu_clk_g),
 .dat_rdy(rdy_wire1),
 .reset(rstconv)
 );
 top conv2(.datain(inp_wire2),
 .dataout(out_wire2),
 .clock(cpu_clk_g),
 .dat_rdy(rdy_wire2),
 .reset(rstconv)
 );
 top conv3(.datain(inp_wire3),
 .dataout(out_wire3),
 .clock(cpu_clk_g),
 .dat_rdy(rdy_wire3),
 .reset(rstconv)
 );

	parameter BUFFER_SIZE = 20'd30000;

  parameter Width = 640;
  parameter FrontH = 106;
  parameter PulseH = 48;
  parameter BackH = 6; //88+5
  parameter Height = 480;
  parameter FrontV = 10;
  parameter PulseV = 2;
  parameter BackV = 33; //23+1

	localparam VGA_IDLE = 1'd0,
						 VGA_ACTIVE = 1'd1;

	reg [23:0] frame_buffer [BUFFER_SIZE-1:0];

  reg [10:0] vga_h_counter, vga_v_counter;
	wire [10:0] vga_j, vga_i;
	assign vga_j = vga_h_counter - (PulseH+BackH);
	assign vga_i = vga_v_counter - (PulseV+BackV);
	//wire [19:0] write_addr;
	//assign write_addr = vga_i*Width + {{9{0}},vga_j};

	wire vga_valid;
	assign vga_valid = (vga_h_counter >= (PulseH+BackH)) &
										 (vga_h_counter <  (PulseH+BackH+Width)) &
										 (vga_v_counter >= (PulseV+BackV)) &
										 (vga_v_counter <  (PulseV+BackV+Height));

//assign fifo_rd_en = !fifo_empty;

     wire [7:0] fifoin1;
	  reg [7:0] fifoinput1;
	  assign fifoin1 = fifoinput1;
	  wire [7:0] fifoin2;
	  reg [7:0] fifoinput2;
	  assign fifoin2 = fifoinput2;
	  wire [7:0] fifoin3;
	  reg [7:0] fifoinput3;
	  assign fifoin3 = fifoinput3;
	  wire [19:0] write_addr;
	  reg [10:0] h_counter,v_counter;
	  wire [10:0] x,y;
	  assign x = h_counter;
	  assign y = v_counter;
	  assign write_addr = y*638 + {{9{0}},x};
	  wire valid_wr2fifo ;
	  reg valid;
	  assign valid_wr2fifo = valid;
	  wire [43:0] fifo_dout;
	  //assign valid_wr2fifo =0;
//
//	fifo_generator_v9_1 fifo(
//	  .rst(rst),
//	  .wr_clk(rdy_wire),
//	  .wr_en(valid_wr2fifo & (write_addr < BUFFER_SIZE)),
//	  .din({write_addr,fifoin,fifoin,fifoin}),
//	  .rd_clk(cpu_clk_g),
//	  .rd_en(fifo_rd_en),
//	  .dout(fifo_dout),
//	  .empty(fifo_empty),
//	  .full(fifo_full));
	  
	  always@(posedge rdy_wire) begin
	  fifoinput1 <= out_wire1;
	  fifoinput2 <= out_wire2;
	  fifoinput3 <= out_wire3;
	  if(rstconv == 1) begin 
	                   h_counter <= 0;
	                   v_counter <= 0;
							 valid <=0;
                      end 
    else 
     begin 
	  valid <= 1;
	  if( (h_counter == 637) && (v_counter == 479))
	     begin 
		  h_counter <= 0;
        v_counter <= 0;
        end
     else if ( h_counter == 637)
	     begin
          h_counter <= 0;
          v_counter <= v_counter + 1;
			 end
     else 
         begin
         h_counter <= h_counter + 1;
          v_counter <= v_counter;
          end	

    end			 
	 if((valid == 1) && (write_addr < BUFFER_SIZE))
	 frame_buffer[write_addr] <= {VGA_IN_RED,VGA_IN_BLUE,VGA_IN_GREEN};
	  
	  end

	reg vga_in_hsout_delayed;
	reg vga_in_vsout_delayed;
	wire vga_in_hsout_posedge;
	wire vga_in_vsout_posedge;
	assign vga_in_hsout_posedge = VGA_IN_HSOUT & !vga_in_hsout_delayed;
	assign vga_in_vsout_posedge = VGA_IN_VSOUT & !vga_in_vsout_delayed;

	reg vga_state;
	always@(posedge VGA_IN_DATA_CLK) begin

		if (rst) begin
		   rst_conv<=1;
			vga_state <= VGA_IDLE;
			vga_in_hsout_delayed <= 1; // to prevent posedge at rst if HSOUT happened to be high
			vga_in_vsout_delayed <= 1; // same
			//vga_h_counter <= 0; // so vga_valid 0 at rst
			//vga_v_counter <= 0;
		end else begin
			vga_in_hsout_delayed <= VGA_IN_HSOUT;
			vga_in_vsout_delayed <= VGA_IN_VSOUT;
			case (vga_state)
				VGA_IDLE: begin
					if (vga_in_vsout_posedge) begin
					//	vga_h_counter <= 0;
					//	vga_v_counter <= 0;
						vga_state <= VGA_ACTIVE;
						rst_conv<=1;
					end
				end
				VGA_ACTIVE: begin
				rst_conv <=0;
				//if(rstconv == 0)
		inp_reg1 <= VGA_IN_RED;
      inp_reg2 <= VGA_IN_GREEN;
      inp_reg3 <= VGA_IN_BLUE;
					if (vga_in_vsout_posedge) begin
						//vga_h_counter <= 0;
						//vga_v_counter <= 0;
						rst_conv<= 1;
					end else if (vga_in_hsout_posedge) begin
						//vga_h_counter <= 0;
						//vga_v_counter <= vga_v_counter + 1;
					end else begin
						//vga_h_counter <= vga_h_counter + 1;
						//vga_v_counter <= vga_v_counter;
					end

				end
				default:
					vga_state <= VGA_IDLE;
			endcase
		end
	end


	reg [23:0] video;
	wire video_next;
	wire video_ready; // it's an output from DVI
	wire video_valid;

	reg [10:0] j, i;
	wire [19:0] read_addr;
	assign read_addr = i*640 + {{9{0}},j};

	always@(posedge cpu_clk_g) begin
		if (rst)
			begin
				j <= 0;
				i <= 0;
				video <= 24'hFFFFFF;
			end
		else
			begin
				if (video_ready)
				begin
				 	if ((j == 639) && (i == 479)) begin
		         j <= 0;
		         i <= 0;
		         end else if (j == 639) begin
		         j <= 0;
		         i <= i + 1;
		         end else begin
		         j <= j + 1;
		         i <= i;
		         end
           end 
		     else 
		      begin
        	   j <= j;
        	   i <= i;
           end
			  if (read_addr < BUFFER_SIZE)
					video <= frame_buffer[read_addr];
				else
					video <= 24'hFFFFFF;
			  
        end


			
			
				
	end
//always@(posedge cpu_clk_g)
//begin
//     
//	  	
//			
//	
//end
	assign video_valid = 1'b1;

  DVI #(
    .ClockFreq(                 25175000),
		.Width(                     800),
		.FrontH(                    16),
		.PulseH(                    96),
		.BackH(                     48),
		.Height(                    525),
		.FrontV(                    10),
		.PulseV(                    2),
		.BackV(                     33)
	) dvi(
		.Clock(                     cpu_clk_g),
		.Reset(                     rst),
		.DVI_D(                     DVI_D),
		.DVI_DE(                    DVI_DE),
		.DVI_H(                     DVI_H),
		.DVI_V(                     DVI_V),
		.DVI_RESET_B(               DVI_RESET_B),
		.DVI_XCLK_N(                DVI_XCLK_N),
		.DVI_XCLK_P(                DVI_XCLK_P),
		.I2C_SCL_DVI(               IIC_SCL_VIDEO),
		.I2C_SDA_DVI(               IIC_SDA_VIDEO),
		/* Ready/Valid interface for 24-bit pixel values */
		.Video(                     video),
		.VideoReady(                video_ready),
		.VideoValid(                video_valid)
	);

  assign GPIO_LED = {1'b1, 1'b1, rst, rst, pll_lock, pll_lock, video_ready, video_ready};

endmodule