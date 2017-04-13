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
output	reg	[7:0]	oSignal,			// Command of EN&STOP
output	reg	[7:0]	oAX_12,
output	reg	[1:0]	okick,			// shoot a ball at the goal
output	reg			oBrush,
output	reg			oRx_done
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	8;
// differentiate state in order to change state
parameter DATA0	=	8'b00000001;
parameter DATA1	=	8'b00000010;
parameter DATA2	=	8'b00000100;
parameter DATA3	=	8'b00001000;
parameter DATA4	=	8'b00010000;
parameter DATA5	=	8'b00100000;
parameter DATA6	=	8'b01000000;
parameter END	=	8'b10000000;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
//	divide information to 6 part and 8 bits per part
reg		[7:0]	rData_0, rData_1, rData_2, rData_3, rData_4, rData_5,rData_6;
reg		[7:0]	rTmpData_0, rTmpData_1, rTmpData_2, rTmpData_3, rTmpData_4, rTmpData_5,rTmpData_6, rTmpData_7;

reg		[SIZE-1:0]	state;

reg				rRx_ready;
reg				rCheck;
reg				rError;
reg		[7:0]	rChecksum;
reg		[7:0]	null = 8'h0;
//=============================================================================
// Structural coding
//=============================================================================

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
		rTmpData_0	<= 	0;
		rTmpData_1	<= 	0;
		rTmpData_2	<= 	0;
		rTmpData_3	<= 	0;
		rTmpData_4	<= 	0;
		rTmpData_5	<= 	0;
		rTmpData_6	<= 	0;
		rTmpData_7	<= 	0;
		state		<=	DATA0;
	end
	// Take apart Data
	else if(~rRx_ready & iRx_ready) begin
		case(state)
			DATA0:
				begin
					rCheck <= 0;
					if( iData == 8'hFF ) begin	// when getting initiation packet, state jump next
						rTmpData_0	<=	iData;
						state	<=	DATA1;
					end
				end
			DATA1:
				begin
					rCheck <= 0;
					if( iData == 8'hFA ) begin	//when getting second initiation packet, start to receive and transmit Data
						rTmpData_1	<=	iData;
						state	<=	DATA2;
					end
					else begin
						state	<=	DATA0;		
					end
				end
			DATA2:				
				begin
					rCheck <= 0;
					rTmpData_2	<=	iData;		//motor1
					state	<=	DATA3;
				end
			DATA3:				
				begin
					rCheck <= 0;
					rTmpData_3	<=	iData;		//motor2
					state	<=	DATA4;
				end
			DATA4:
				begin
					rCheck <= 0;
					rTmpData_4	<=	iData;		//motor3
					state	<=	DATA5;
				end
			DATA5:
				begin
					rCheck <= 0;
					rTmpData_5	<=	iData;		//enable+stop
					state	<=	DATA6;
				end
			DATA6:
				begin
					rCheck <= 0;
					rTmpData_6	<=	iData;		//shoot
					state	<=	END;
				end
			END:
				begin
					rCheck <= 1;
					rTmpData_7	<=	iData;		//checksum
					oRx_done	<=	1;
					state	<=	DATA0;
				end
					

		endcase
	end
	else begin
		rChecksum <= rTmpData_2 + rTmpData_3 + rTmpData_4 + rTmpData_5 + rTmpData_6;
		if((rChecksum == rTmpData_7) && (rTmpData_0 == 8'hFF) && (rTmpData_1 == 8'hFA))begin
			rError <= 0;
			rData_2 <= rTmpData_2;
			rData_3 <= rTmpData_3;
			rData_4 <= rTmpData_4;
			rData_5 <= rTmpData_5;
			rData_6 <= rTmpData_6;
		end
		else begin
			rError <= 1;
			rData_2 <= rData_2;
			rData_3 <= rData_3;
			rData_4 <= rData_4;
			rData_5 <= rData_5;
			rData_6 <= rData_6;
		end
	end
	
	oCMD_Motor1	<=	rData_2;			// give oCMD_Motor1 Data
	oCMD_Motor2	<=	rData_3;			// give oCMD_Motor2 Data
	oCMD_Motor3	<=	rData_4;			// give oCMD_Motor3 Data
	oSignal		<=	{rData_5[7:2],null[1:0]};			// give oSignal Data (Enable + Stop)
	okick		<=	rData_6;		// shoot a ball at the goal
	oRx_done	<=	0;

	rRx_ready	<=	iRx_ready;
end

endmodule
