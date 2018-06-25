// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert serial to command
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/07/19 :|  Initial Version
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.8  :| Chun-Jui Huang    :| 2017/07/07 :|  Add Checksum and Shoot Control
// --------------------------------------------------------------------

module Serial2CMD (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 		// 50MHz
input				iRst_n,		// Reset
input				iRx_ready,
input		[7:0]	iData,		// Data
output	reg	[7:0]	oCMD_Motor1,	// Command of motor1
output	reg	[7:0]	oCMD_Motor2,	// Command of motor2
output	reg	[7:0]	oCMD_Motor3,	// Command of motor3
output	reg	[7:0]	oCMD_Motor4,	// Command of motor4
output	reg	[7:0]	oSignal,			// Command of EN&STOP
output	reg	[7:0]	oAX_12,
output	reg	[7:0]	okick,			// shoot a ball at the goal
output	reg			oBrush,
output	reg			oRx_done,
output	reg			rError,
output	reg	[71:0]all
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	12;
// differentiate state in order to change state
parameter DATA0	=	12'b000000000001;
parameter DATA1	=	12'b000000000010;
parameter DATA2	=	12'b000000000100;
parameter DATA3	=	12'b000000001000;
parameter DATA4	=	12'b000000010000;
parameter DATA5	=	12'b000000100000;
parameter DATA6	=	12'b000001000000;
parameter DATA7	=	12'b000010000000;
parameter DATA8	=	12'b000100000000;
parameter DATA9	=	12'b001000000000;
parameter DATA10	=	12'b010000000000;
parameter END		=	12'b100000000000;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
//	divide information to 6 part and 8 bits per part
reg		[7:0]	rData_0, rData_1, rData_2, rData_3, rData_4, rData_5,rData_6,rData_7,rData_8,rData_9,rData_10,rData_11,rData_12;
reg		[7:0]	rTmpData_0, rTmpData_1, rTmpData_2, rTmpData_3, rTmpData_4, rTmpData_5, rTmpData_6, rTmpData_7, rTmpData_8, rTmpData_9;



reg		[SIZE-1:0]	state;

reg				rRx_ready;
reg				rCheck;
reg		[7:0]	rChecksum;
reg		[7:0]	null = 8'h0;
//=============================================================================
// Structural coding
//=============================================================================

integer i,j;
reg [71:0]crc;

always @(posedge iCLK) begin
	if(!iRst_n) begin		// Reset
		oRx_done	<=	0;
		rData_0		<=	0;
		rData_1		<=	0;
		rData_2		<=	0;
		rData_3		<=	0;
		rData_4		<=	0;
		rData_5		<=	0;
		rData_6		<=	0;
		rData_7		<=	0;
		rData_8		<=	0;
		rData_9		<=	0;
		rData_10		<=	0;
		rData_11		<=	0;
		rData_12		<=	0;
		state			<=	DATA0;
		all			<= 0;
		oCMD_Motor1	<=	0;
		oCMD_Motor2	<=	0;
		oCMD_Motor3	<=	0;
		oCMD_Motor4	<=	0;
		oSignal		<=	0;
		okick			<=	0;
		rError		<=	1;
		rChecksum	<=	0;
		rTmpData_0	<=	0;
		rTmpData_1	<=	0;
		rTmpData_2	<=	0;
		rTmpData_3	<=	0;
		rTmpData_4	<=	0;
		rTmpData_5	<=	0;
		rTmpData_6	<=	0;
		rTmpData_7	<=	0;
	end
	// Take apart Data
	else begin
		
		rChecksum <= rData_0 + rData_1 + rData_2 + rData_3 + rData_4 + rData_5 + rData_6 + rData_7 + rData_8;

		if(~rRx_ready & iRx_ready) begin
			case(state)
				DATA0:
//					begin
//						if( iData == 8'hFF ) begin	// when getting initiation packet, state jump next
//							rData_0	<=	iData;
//							state	<=	DATA1;
//						end
//					end
					begin
						if( iData == 8'h55 ) begin	// when getting initiation packet, state jump next
							rData_0	<=	iData;
							state	<=	DATA1;
						end
					end
				DATA1:
//					begin
//						if( iData == 8'hFA ) begin	//when getting second initiation packet, start to receive and transmit Data
//							rData_1	<=	iData;
//							state	<=	DATA2;
//						end
//						else begin
//							state	<=	DATA0;		
//						end
//					end
					begin
						if( iData == 8'hAA ) begin	//when getting second initiation packet, start to receive and transmit Data
							rData_1	<=	iData;
							state	<=	DATA2;
						end
						else begin
							state	<=	DATA0;		
						end
					end
				DATA2:				
					begin
						rData_2	<=	iData;		//motor1
						state	<=	DATA3;
					end
				DATA3:				
					begin
						rData_3	<=	iData;		//motor2
						state	<=	DATA4;
					end
				DATA4:
					begin
						rData_4	<=	iData;		//motor3
						state	<=	DATA5;
					end
				DATA5:
					begin
						rData_5	<=	iData;		//motor4
						state	<=	DATA6;
					end
				DATA6:
					begin
						rData_6	<=	iData;		//enable+stop
						state	<=	DATA7;
					end
				DATA7:
					begin
//						rCheck <= 0;
						rData_7	<=	iData;		//crc16-1
						state	<=	DATA8;
					end
				DATA8:
					begin
						rData_8	<=	iData;		//crc16-2
						all <={rData_2,rData_3,rData_4,rData_5,rData_6,rData_7,rData_8};
						crc ={rData_0, rData_1, rData_2,rData_3,rData_4,rData_5,rData_6,rData_7,rData_8};
						for(i=0; i<56; i=i+1)begin
							if(crc&71'h100000000000000000 == 71'h100000000000000000)begin
								crc = (crc << 1);
								crc[71:56] = crc[71:56] ^ 16'h1021;
							end
							else begin
								crc = (crc << 1);
							end
						end
						state	<=	DATA9;
					end
				DATA9:
					begin
						rData_9	<=	iData;		//checksum
						oRx_done	<=	1;
						state	<=	DATA0;
					end
						

			endcase
		end
		else begin
//			oCMD_Motor1 <= rData_2;
//			oCMD_Motor2 <= rData_3;
//			oCMD_Motor3 <= rData_4;
//			oSignal 	<= rData_5;
//			okick 		<= rData_6;
//			
			rData_0 <= rData_0;
			rData_1 <= rData_1;
			rData_2 <= rData_2;
			rData_3 <= rData_3;
			rData_4 <= rData_4;
			rData_5 <= rData_5;
			rData_6 <= rData_6;
			rData_7 <= rData_7;
			rData_8 <= rData_8;
			rData_9 <= rData_9;
			all	  <= all;
			rTmpData_0	<=	rData_0;
			rTmpData_1	<=	rData_1;
			rTmpData_2	<=	rData_2;
			rTmpData_3	<=	rData_3;
			rTmpData_4	<=	rData_4;
			rTmpData_5	<=	rData_5;
			rTmpData_6	<=	rData_6;
			rTmpData_7	<=	rData_7;
//			if (rData_7 == 1)begin    // for shoot cmd
//				rTmpData_7	<=	0;
//			end
//			else begin
//				rTmpData_7	<=	rData_7;
//			end
			rTmpData_8 	<=	rData_8;
			rTmpData_9  <= rData_9; 
			state <= state;
			all[7:0] <= rChecksum;
//			crc ={rData_2,rData_3,rData_4,rData_5,rData_6,rData_7,rData_8};
//			for(i=0; i<40; i=i+1)begin
//				if(crc&56'h10000000000000 == 56'h10000000000000)begin
//					crc = (crc << 1);
//					crc[55:40] = crc[55:40] ^ 16'h1021;
//				end
//				else begin
//					crc = (crc << 1);
//				end
//			end
//			all[15:0] <= crc[55:40];
						
//			if((rTmpData_7 == crc[71:64]) && (rTmpData_8 == crc[63:56]) && (rTmpData_0 == 8'hFF) && (rTmpData_1 == 8'hFA) && (rTmpData_9 == rChecksum) ) begin
			if((rTmpData_7 == crc[71:64]) && (rTmpData_8 == crc[63:56]) && (rTmpData_0 == 8'h55) && (rTmpData_1 == 8'hAA) && (rTmpData_9 == rChecksum) ) begin
				rError <= 0;
				oCMD_Motor1 <= rTmpData_2;
				oCMD_Motor2 <= rTmpData_3;
				oCMD_Motor3 <= rTmpData_4;
				oCMD_Motor4 <= rTmpData_5;
				oSignal 		<= rTmpData_6;
				okick 		<= rTmpData_7;
			end
			else begin
				//all[7:0] <= 0;
				rError <= 1;
				oCMD_Motor1 	<= 	oCMD_Motor1;
				oCMD_Motor2 	<= 	oCMD_Motor2;
				oCMD_Motor3 	<= 	oCMD_Motor3;
				oCMD_Motor4 	<= 	oCMD_Motor4;
				oSignal 		<= 	oSignal;
				okick			<= 	okick;
			end
			oRx_done	<=	0;
		end
		
		rRx_ready	<=	iRx_ready;
	end
end


endmodule
