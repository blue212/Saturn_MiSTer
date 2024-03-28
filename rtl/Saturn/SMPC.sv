module SMPC (
	input              CLK,
	input              RST_N,
	input              CE,
	
	input              MRES_N,
	input              TIME_SET,
	
	input      [ 3: 0] AC,
	
	input      [ 6: 1] A,
	input      [ 7: 0] DI,
	output     [ 7: 0] DO,
	input              CS_N,
	input              RW_N,
	
	input              SRES_N,
	
	input              IRQV_N,
	input              EXL,
	
	output reg         MSHRES_N,
	output reg         MSHNMI_N,
	output reg         SSHRES_N,
	output reg         SSHNMI_N,
	output reg         SYSRES_N,
	output reg         SNDRES_N,
	output reg         CDRES_N,
	
	output reg         MIRQ_N,

	output reg         DOTSEL,
	
	input      [ 6: 0] PDR1I,
	output reg [ 6: 0] PDR1O,
	output reg [ 6: 0] DDR1,
	input      [ 6: 0] PDR2I,
	output reg [ 6: 0] PDR2O,
	output reg [ 6: 0] DDR2,
	
	output reg         INPUT_ACT,
	output     [ 4: 0] INPUT_POS,
	input      [ 7: 0] INPUT_DATA,
	input              INPUT_WE,
	
	input      [ 6: 0] USERJOYSTICK,
	output     [ 6: 0] USERJOYSTICKOUT,
	input              snac,
	//input              joyswap,	
	input              VBL_N,
	input      [ 2: 0] JOY1_TYPE
);

	//Registers
	bit   [7:0] COMREG;
	bit   [7:0] SR;
	bit         SF;
	bit   [7:0] IREG[7];
	bit         PDR1O7;
	bit         PDR2O7;
	bit   [1:0] IOSEL;
//	bit   [1:0] EXLE;
	
	bit          RESD;
	bit          STE;
	
	bit  [ 7: 0] SEC;
	bit  [ 7: 0] MIN;
	bit  [ 7: 0] HOUR;
	bit  [ 7: 0] DAYS;
	bit  [ 3: 0] DAY;
	bit  [ 3: 0] MONTH;
	bit  [15: 0] YEAR;
	
//	bit  [ 7: 0] SMEM[4];

	parameter SR_PDE = 2;
	parameter SR_RESB = 3;
	
	typedef enum bit [3:0] {
		CS_IDLE,
		CS_WAIT, 
		CS_COMMAND,
		CS_RESET, 
		CS_EXEC,
		CS_INTBACK_PERI,
		CS_INTBACK_BREAK, 
		CS_END
	} CommExecState_t;
	CommExecState_t COMM_ST;

	//snac
	bit   [15:0] serJoy;
	bit   [ 6:0] UserSel;
	bit   [15:0] tempdata;
	bit   [ 3:0] tempbits;	
	bit   [14:0] temp_CNT;
	bit   [14:0] temp_CNT2;
	bit   [14:0] snac_CNT;
	bit   [ 4:0] data_CNT;
	bit   [ 7:0] UserData[11];	//enough for dual mission stick
	bit   [ 3:0] DataLUT[6];
	bit   oldVBL;
	bit   [ 3:0] megaID;
	bit   THTRmode;
	bit   threelinemode;
	bit   THmode;
	bit   waitforTLrising;
	bit   waitforTLfalling;	
	bit   oldTL;
	bit   [ 7:0] saturnID;
	bit   [ 4:0] dataSize1;
	bit   [ 4:0] port2Start;

	assign {DataLUT[0], DataLUT[1], DataLUT[2], DataLUT[3], DataLUT[4], DataLUT[5]} = {4'd4, 4'd1, 4'd5, 4'd7, 4'd8, 4'd11 };//data size of JOY1_TYPE	
	
	always_comb begin	
		if (snac) USERJOYSTICKOUT = IOSEL[0] ? ~DDR1 | PDR1O : UserSel;		
		//else if (snac && joyswap) USERJOYSTICKOUT = IOSEL[1] ? ~DDR2 | PDR2O : UserSel;
		else USERJOYSTICKOUT = 7'h7F;		
	end
	
	always @(posedge CLK) begin
		tempdata = serJoy;
		oldVBL <= VBL_N;		
		if (~oldVBL && VBL_N) begin//start counter
			snac_CNT <= 15'd0;
			waitforTLfalling <= 1'b0;
			waitforTLrising <= 1'b0;			
			THTRmode <= 1'b0;
			threelinemode <= 1'b0;
			THmode <= 1'b0;
			//segatap <= 1'b0;
			//multitap <= 1'b0;
			UserSel = 7'b1111111;
		end		
		if (snac_CNT < 15'd32001) snac_CNT <= snac_CNT + 1'd1;
	
		//check for something hooked up every frame by toggling TH low for 80us		
		// calcualte mega drive id from that		
		//determine protocol from that. saturn id only gotten if md id is 0x5		
		//timings of select doesn't match real HW

		if (snac_CNT == 15'd1000) serJoy[3:0] = USERJOYSTICK[3:0];
		else if (snac_CNT == 15'd2500) {UserSel[6],UserSel[5]} <= 2'b01;//s0(TH) low, s1(TR) high. RLDU
		else if (snac_CNT == 15'd3500) begin
			serJoy[15:12] = USERJOYSTICK[3:0];				
			megaID <= {serJoy[3] | serJoy[2], serJoy[1] | serJoy[0], USERJOYSTICK[3] | USERJOYSTICK[2], USERJOYSTICK[1] | USERJOYSTICK[0]};//calculate id1
		end else if (snac_CNT == 15'd3501) begin//get type
			if (megaID == 4'hB) begin//TH and TR control mode							
				THTRmode <= 1'b1;
				UserData[0] <= 8'hF1;//portstatus1;
				UserData[1] <= 8'h02;//saturnID; 
				port2Start <= 5'd4;
			end else if (megaID == 4'h5) begin//3 line handshake - have to get saturn id
				threelinemode <= 1'b1;
				dataSize1 <= 5'd3;//temp value
				UserData[0] <= 8'hF1;// portstatus1;//taps aren't F1
			/*
			end else if (megaID == 4'h7) begin//sega multitap (GEN/MD)					
				threelinemode <= 1'b1;
				segatap <= 1'b1;
				portstatus1 <= 8'h04;
			*/					
			end else if (megaID == 4'h3) begin //mouse					
				threelinemode <= 1'b1;
				dataSize1 <= 5'd3;
				port2Start <= 5'd5;
				UserData[0] <= 8'hF1;//portstatus1;
				UserData[1] <= 8'hE3;//saturnID; 	
			end else if (megaID == 4'hD) begin//GEN/MD 3/6					
				THmode <= 1'b1;
				UserData[0] <= 8'hF1;//portstatus1;
			end else if (megaID == 4'hF) begin//nothing detected					
				THTRmode <= 1'b0;
				threelinemode <= 1'b0;
				THmode <= 1'b0;
			end
		end
		
		if (~THTRmode && ~threelinemode && ~THmode) begin
			if (snac_CNT == 15'd5000) begin
				{UserSel[6],UserSel[5]} <= 2'b11;
				UserData[0] <= {megaID , 4'h0};//F0 = nothing, A0 = Stunner
				port2Start <= 5'd1;	
			end	
		end

		////////////TH and TR control mode
		if (THTRmode) begin
			if (snac_CNT == 15'd5000) {UserSel[6],UserSel[5]} <= 2'b10;//Sacb
			else if (snac_CNT == 15'd6500) serJoy[11:8] = USERJOYSTICK[3:0];
			else if (snac_CNT == 15'd7500) {UserSel[6],UserSel[5]} <= 2'b00;//rxyz
			else if (snac_CNT == 15'd9000) serJoy[7:4] = USERJOYSTICK[3:0];
			else if (snac_CNT == 15'd10000) begin
				{UserSel[6],UserSel[5]} <= 2'b11;
				serJoy[2:0] = 3'b111;//fix the bits
			end else if (snac_CNT == 15'd10001) begin
				UserData[2] <= tempdata[15:8]; 
				UserData[3] <= tempdata[7:0]; 					
			end			
		end
				
		//////////// TH mode
		if (THmode) begin
			if (snac_CNT == 15'd5000) {UserSel[6],UserSel[5]} <= 2'b11;
			else if (snac_CNT == 15'd5500) begin				
				serJoy[15:12] = USERJOYSTICK[3:0];//RLDU
				serJoy[9:8] = USERJOYSTICK[5:4];//cb					
			end else if (snac_CNT == 15'd6000) {UserSel[6],UserSel[5]} <= 2'b01;
			else if (snac_CNT == 15'd6500) serJoy[11:10] = USERJOYSTICK[5:4];//SA					
			else if (snac_CNT == 15'd7000) {UserSel[6],UserSel[5]} <= 2'b11;
			else if (snac_CNT == 15'd7500) serJoy[3:0] = USERJOYSTICK[3:0];//for id check
			else if (snac_CNT == 15'd8000) {UserSel[6],UserSel[5]} <= 2'b01;
			else if (snac_CNT == 15'd9000) megaID <= {serJoy[3] | serJoy[2], serJoy[1] | serJoy[0], USERJOYSTICK[3] | USERJOYSTICK[2], USERJOYSTICK[1] | USERJOYSTICK[0]};//check id again
			else if (snac_CNT == 15'd10000) begin			
				{UserSel[6],UserSel[5]} <= 2'b11;
				if (megaID == 4'hD) begin
					saturnID <= 8'hE1;//3 button
					port2Start <= 5'd3;
					UserData[1] <= 8'hE1;//saturnID;
					UserData[2] <= tempdata[15:8];
				end else if (megaID == 4'hC) begin
					saturnID <= 8'hE2;//6 button
					UserData[1] <= 8'hE2;//saturnID;
					port2Start <= 5'd4;	
				end
			end			
			if (saturnID == 8'hE2) begin///6 button
				if (snac_CNT == 15'd11000) serJoy[7:4] = USERJOYSTICK[3:0];//MXYZ
				else if (snac_CNT == 15'd12000) {UserSel[6],UserSel[5]} <= 2'b01;	
				else if (snac_CNT == 15'd13000) serJoy[3:0] = USERJOYSTICK[3:0];//1111						
				else if (snac_CNT == 15'd14000) begin			
					{UserSel[6],UserSel[5]} <= 2'b11;
					UserData[2] <= tempdata[15:8]; 
					UserData[3] <= tempdata[7:0];							
				end
			end
		end		
	
		//////////////threeline mode
		if (threelinemode) begin			
			oldTL <= USERJOYSTICK[4];	
			if (snac_CNT == 15'd5000) begin	
				{UserSel[6],UserSel[5]} <= 2'b00;				
				waitforTLfalling <= 1'b1;
				data_CNT <= 5'd0;
			end else if (waitforTLrising && snac_CNT == temp_CNT + 15'd800) begin				
				{UserSel[6],UserSel[5]} <= 2'b01;
			end else if (waitforTLfalling && snac_CNT == temp_CNT2 + 15'd1000) begin				
				{UserSel[6],UserSel[5]} <= 2'b00;
				snac_CNT <= 15'd10000;//reset the counter to 10000 so it doesn't get too big
			end else if (waitforTLfalling) begin
				if (oldTL && ~USERJOYSTICK[4]) begin
					if (data_CNT == 5'd0) begin
						saturnID[7:4] = USERJOYSTICK[3:0];//half of saturn id
					end else begin
						tempbits <= USERJOYSTICK[3:0];//savings half the bits til rising edge
					end						
					temp_CNT <= snac_CNT;
					waitforTLfalling <= 1'b0;
					waitforTLrising <= 1'b1;							
				end
			end else if (waitforTLrising) begin
				if (~oldTL && USERJOYSTICK[4]) begin						
					if (data_CNT == 5'd0) begin
						if (megaID != 4'h3 && megaID != 4'h7) begin
							saturnID[3:0] = USERJOYSTICK[3:0];//half of saturn id, datasize
							dataSize1 <= {1'b0,USERJOYSTICK[3:0]};
							port2Start <= 5'd2 + USERJOYSTICK[3:0];
							UserData[1] <= {saturnID[7:4],USERJOYSTICK[3:0]};
						end	
					end else begin
						UserData[data_CNT + 1] <= {tempbits, USERJOYSTICK[3:0]};
					end					
					data_CNT <= data_CNT + 5'd1;
					temp_CNT2 <= snac_CNT;
					waitforTLrising <= 1'b0;
					if (data_CNT < dataSize1) waitforTLfalling <= 1'b1;
					else {UserSel[6],UserSel[5]} <= 2'b11;
				end
			end
		end
	end
	/////////
				
	always @(posedge CLK or negedge RST_N) begin
		bit [21: 0] CLK_CNT;
		bit         SEC_CLK,MIN_CLK,HOUR_CLK,DAYS_CLK,MONTH_CLK,YEAR_CLK;
		
		if (!RST_N) begin
			SEC <= 8'h00;
			MIN <= 8'h00;
			HOUR <= 8'h00;
			DAYS <= 8'h01;
			{DAY,MONTH} <= 8'h01;
			YEAR <= 16'h2024;
		end else if (COMM_ST == CS_EXEC && COMREG == 8'h16) begin
`ifndef DEBUG
			SEC <= IREG[6];
			MIN <= IREG[5];
			HOUR <= IREG[4];
			DAYS <= IREG[3];
			{DAY,MONTH} <= IREG[2];
			YEAR <= {IREG[0],IREG[1]};
`endif
		end else if (CE) begin
`ifndef DEBUG
			SEC_CLK <= 0;
			MIN_CLK <= 0;
			HOUR_CLK <= 0;
			DAYS_CLK <= 0;
			MONTH_CLK <= 0;
			YEAR_CLK <= 0;
				
			CLK_CNT <= CLK_CNT + 3'd1;
			if (CLK_CNT == 22'd4000000-1) begin
				CLK_CNT <= 22'd0;
				SEC_CLK <= 1;
			end
			
			if (SEC_CLK) begin
				SEC[3:0] <= SEC[3:0] + 4'd1;
				if (SEC[3:0] == 4'd9) begin
					SEC[3:0] <= 4'd0;
					SEC[7:4] <= SEC[7:4] + 4'd1;
					if (SEC[7:4] == 4'd5) begin
						SEC[7:4] <= 4'd0;
						MIN_CLK <= 1;
					end
				end
			end
			if (MIN_CLK) begin
				MIN[3:0] <= MIN[3:0] + 4'd1;
				if (MIN[3:0] == 4'd9) begin
					MIN[3:0] <= 4'd0;
					MIN[7:4] <= MIN[7:4] + 4'd1;
					if (MIN[7:4] == 4'd5) begin
						MIN[7:4] <= 4'd0;
						HOUR_CLK <= 1;
					end
				end
			end
			if (HOUR_CLK) begin
				HOUR[3:0] <= HOUR[3:0] + 4'd1;
				if (HOUR[3:0] == 4'd9) begin
					HOUR[3:0] <= 4'd0;
					HOUR[7:4] <= HOUR[7:4] + 4'd1;
				end
				else if (HOUR == 8'h23) begin
					HOUR <= 8'h00;
					DAYS_CLK <= 1;
				end
			end
			if (DAYS_CLK) begin
				DAYS[3:0] <= DAYS[3:0] + 4'd1;
				if (DAYS[3:0] == 4'd9) begin
					DAYS[7:4] <= DAYS[7:4] + 4'd1;
					DAYS[3:0] <= 4'd0;
				end
				else if ((DAYS == 8'h28 && MONTH == 4'd2) || 
							(DAYS == 8'h30 && MONTH == 4'd4) || 
							(DAYS == 8'h30 && MONTH == 4'd6) || 
							(DAYS == 8'h30 && MONTH == 4'd9) || 
							(DAYS == 8'h30 && MONTH == 4'd11) || 
							 DAYS == 8'h31) begin
					DAYS <= 8'h01;
					MONTH_CLK <= 1;
				end
			end
			if (MONTH_CLK) begin
				MONTH <= MONTH + 4'd1;
				if (MONTH == 4'd12) begin
					MONTH <= 4'd1;
					YEAR_CLK <= 1;
				end
			end
			if (YEAR_CLK) begin
				YEAR <= YEAR + 16'd1;
			end
`endif
		end
	end
	
	
	bit [ 7: 0] REG_DO;
	bit [ 4: 0] OREG_CNT;
	always @(posedge CLK or negedge RST_N) begin
		CommExecState_t NEXT_COMM_ST;
		bit        VBLANK_PEND;
		bit        COMREG_SET;
		bit        INTBACK_BREAK_PEND;
		bit        RW_N_OLD;
		bit        CS_N_OLD;
		bit        IRQV_N_OLD;
		bit [15:0] WAIT_CNT;
		bit [15:0] INTBACK_WAIT_CNT;
		bit        SRES_EXEC;
		bit        INTBACK_EXEC;
		bit        INTBACK_PERI;
		bit        INTBACK_OPTIM_EN;
		bit        INTBACK_OPTIM_COND;
		bit        CHECK_CONTINUE;
		bit        BREAK,CONT,CONT_PREV;
		
		if (!RST_N) begin
			COMREG <= '0;
			SR <= '0;
			SF <= 0;
			IREG <= '{7{'0}};
			PDR1O <= '0;
			PDR2O <= '0;
			DDR1 <= '0;
			DDR2 <= '0;
			IOSEL <= '0;
//			EXLE <= '0;
			
			MSHRES_N <= 0;
			MSHNMI_N <= 0;
			SSHRES_N <= 0;
			SSHNMI_N <= 0;
			SYSRES_N <= 0;
			SNDRES_N <= 0;
			CDRES_N <= 0;
			MIRQ_N <= 1;
			DOTSEL <= 0;
			RESD <= 1;
			STE <= 0;
			
			REG_DO <= '0;
			RW_N_OLD <= 1;
			CS_N_OLD <= 1;
			IRQV_N_OLD <= 1;
			COMM_ST <= CS_IDLE;
			SRES_EXEC <= 0;
			INTBACK_EXEC <= 0;
			INTBACK_PERI <= 0;
			BREAK <= 0;
			CONT <= 0;
			CONT_PREV <= 0;
			INTBACK_BREAK_PEND <= 0;
			VBLANK_PEND <= 0;
			
			INPUT_ACT <= 0;
		end
		else if (!MRES_N) begin
			MSHRES_N <= 1;
			MSHNMI_N <= 1;
			SSHRES_N <= 0;
			SSHNMI_N <= 1;
			SYSRES_N <= 1;
			SNDRES_N <= 0;
			CDRES_N <= 1;
			MIRQ_N <= 1;
			DOTSEL <= 0;
			SR <= '0;
			RESD <= 1;
			STE <= TIME_SET;/////////////////
			
			COMM_ST <= CS_IDLE;
			INTBACK_EXEC <= 0;
			INTBACK_PERI <= 0;
			INTBACK_OPTIM_COND <= 0;
			BREAK <= 0;
			CONT <= 0;
			CONT_PREV <= 0;
			CHECK_CONTINUE <= 0;
			INTBACK_BREAK_PEND <= 0;
			VBLANK_PEND <= 0;
		end else begin
			OREG_RAM_WE <= 0;
			
			if (CE) begin
				IRQV_N_OLD <= IRQV_N;
				
				if (WAIT_CNT) WAIT_CNT <= WAIT_CNT - 16'd1;
				
				if (!SRES_N && !RESD && !SRES_EXEC) begin
					MSHNMI_N <= 0;
					SSHNMI_N <= 0;
					WAIT_CNT <= 16'd60000;
					SRES_EXEC <= 1;
				end else if (SRES_EXEC && !WAIT_CNT) begin
					MSHNMI_N <= 1;
					SSHNMI_N <= 1;
				end
				
				if (!IRQV_N) begin
					INTBACK_OPTIM_COND <= 0;
					INTBACK_WAIT_CNT <= 16'd52200;
				end
				else if (!INTBACK_WAIT_CNT) begin
					INTBACK_OPTIM_COND <= 1;
				end
				else begin
					INTBACK_WAIT_CNT <= INTBACK_WAIT_CNT - 16'd1;
				end
				
				SR[4:0] <= {~SRES_N,IREG[1][7:4]};
				
				case (COMM_ST)
					CS_IDLE: begin
						if (INTBACK_EXEC && (INTBACK_BREAK_PEND || VBLANK_PEND)) begin
							INTBACK_BREAK_PEND <= 0;
							WAIT_CNT <= 16'd70;
							NEXT_COMM_ST <= CS_INTBACK_BREAK;
							COMM_ST <= CS_WAIT;
						end 
						else if (INTBACK_PERI && ((INTBACK_OPTIM_EN && INTBACK_OPTIM_COND) || !INTBACK_OPTIM_EN) && IRQV_N && !SRES_EXEC) begin
							OREG_CNT <= '0;
							WAIT_CNT <= INTBACK_OPTIM_EN ? 16'd70 : 16'd2000;
							NEXT_COMM_ST <= CS_INTBACK_PERI;
							COMM_ST <= CS_WAIT;
						end 
						else if (COMREG_SET && !SRES_EXEC) begin
							COMREG_SET <= 0;
							OREG_CNT <= '0;
							WAIT_CNT <= 16'd90;
							NEXT_COMM_ST <= CS_COMMAND;
							COMM_ST <= CS_WAIT;
						end
						VBLANK_PEND <= 0;
						MIRQ_N <= 1;
						
						if (CHECK_CONTINUE) begin
							if (BREAK) begin
								INTBACK_BREAK_PEND <= 1;
								BREAK <= 0;
								CHECK_CONTINUE <= 0;
							end
							else if (CONT != CONT_PREV) begin
								INTBACK_PERI <= 1;
								SF <= 1;
								CONT_PREV <= CONT;
								CHECK_CONTINUE <= 0;
							end
						end
					end
					
					CS_WAIT: begin
						if (!WAIT_CNT) begin
							if (NEXT_COMM_ST == CS_INTBACK_PERI) INPUT_ACT <= 1;
							COMM_ST <= NEXT_COMM_ST;
						end
					end
					
					CS_COMMAND: begin
						case (COMREG) 
							8'h00: begin		//MSHON
								WAIT_CNT <= 16'd120;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h02: begin		//SSHON
								WAIT_CNT <= 16'd120;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h03: begin		//SSHOFF
								WAIT_CNT <= 16'd120;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h06: begin		//SNDON
								WAIT_CNT <= 16'd120;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h07: begin		//SNDOFF
								WAIT_CNT <= 16'd120;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h08: begin		//CDON
								WAIT_CNT <= 16'd159;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h09: begin		//CDOFF
								WAIT_CNT <= 16'd159;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h0D: begin		//SYSRES
								if (IRQV_N && !IRQV_N_OLD) begin
									MSHRES_N <= 0;
									MSHNMI_N <= 0;
									SSHRES_N <= 0;
									SSHNMI_N <= 0;
									SNDRES_N <= 0;
									CDRES_N <= 0;
									SYSRES_N <= 0;
									COMM_ST <= CS_RESET;
								end
							end
							
							8'h0E: begin		//CKCHG352
								if (IRQV_N && !IRQV_N_OLD) begin
									SSHRES_N <= 0;
									SSHNMI_N <= 0;
									SNDRES_N <= 0;
									SYSRES_N <= 0;
									DOTSEL <= 1;
									COMM_ST <= CS_RESET;
								end
							end
							
							8'h0F: begin		//CKCHG320
								if (IRQV_N && !IRQV_N_OLD) begin
									SSHRES_N <= 0;
									SSHNMI_N <= 0;
									SNDRES_N <= 0;
									SYSRES_N <= 0;
									DOTSEL <= 0;
									COMM_ST <= CS_RESET;
								end
							end
							
							8'h10: begin		//INTBACK
								if (IREG[2] == 8'hF0 && (IREG[0][0] || IREG[1][3])) begin
									if (IREG[0][0]) begin
										WAIT_CNT <= 16'd800;
										NEXT_COMM_ST <= CS_EXEC;
										COMM_ST <= CS_WAIT;
									end else begin
										INTBACK_EXEC <= 1;
										INTBACK_PERI <= 1;
										INTBACK_OPTIM_EN <= ~IREG[1][1];
										CONT_PREV <= 0;
										COMM_ST <= CS_IDLE;
									end
								end else begin
									COMM_ST <= CS_END;
								end
							end
							
							8'h16: begin		//SETTIME
								WAIT_CNT <= 16'd279;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h17: begin		//SETSMEM
								WAIT_CNT <= 16'd159;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h18: begin		//NMIREQ
								WAIT_CNT <= 16'd127;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h19: begin		//RESENAB
								WAIT_CNT <= 16'd127;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							8'h1A: begin		//RESDISA
								WAIT_CNT <= 16'd127;
								NEXT_COMM_ST <= CS_EXEC;
								COMM_ST <= CS_WAIT;
							end
							
							default: begin
								COMM_ST <= CS_EXEC;
							end
						endcase
					end
					
					CS_RESET: begin
						case (COMREG) 
							8'h0D: begin		//SYSRES
								CDRES_N <= 1;
								SNDRES_N <= 1;
								SYSRES_N <= 1;
							end
							
							8'h0E: begin		//CKCHG352
								SYSRES_N <= 1;
							end
							
							8'h0F: begin		//CKCHG320
								SYSRES_N <= 1;
							end
							
							default: ;
						endcase
						
						if (!IRQV_N && IRQV_N_OLD) begin
							WAIT_CNT <= 16'd4000;
							NEXT_COMM_ST <= CS_EXEC;
							COMM_ST <= CS_WAIT;
						end
					end
					
					CS_EXEC: begin
						OREG_RAM_WA <= 5'd31;
						OREG_RAM_D <= COMREG;
						OREG_RAM_WE <= 1;
						case (COMREG) 
							8'h00: begin		//MSHON
								MSHRES_N <= 1;
								MSHNMI_N <= 1;//?
								COMM_ST <= CS_END;
							end
							
							8'h02: begin		//SSHON
								SSHRES_N <= 1;
								SSHNMI_N <= 1;//?
								COMM_ST <= CS_END;
							end
							
							8'h03: begin		//SSHOFF
								SSHRES_N <= 0;
								SSHNMI_N <= 1;//?
								COMM_ST <= CS_END;
							end
							
							8'h06: begin		//SNDON
								SNDRES_N <= 1;
								COMM_ST <= CS_END;
							end
							
							8'h07: begin		//SNDOFF
								SNDRES_N <= 0;
								COMM_ST <= CS_END;
							end
							
							8'h08: begin		//CDON
								CDRES_N <= 1;
								COMM_ST <= CS_END;
							end
							
							8'h09: begin		//CDOFF
								CDRES_N <= 0;
								COMM_ST <= CS_END;
							end
							
							8'h0D: begin		//SYSRES
								COMM_ST <= CS_END;
							end
							
							8'h0E: begin		//CKCHG352
								MSHNMI_N <= 0;
								COMM_ST <= CS_END;
							end
							
							8'h0F: begin		//CKCHG320
								MSHNMI_N <= 0;
								COMM_ST <= CS_END;
							end
							
							8'h10: begin		//INTBACK
								if (!INTBACK_EXEC) begin
									OREG_RAM_WA <= OREG_CNT;
									case (OREG_CNT)
										5'd0: OREG_RAM_D <= {STE,RESD,6'b000000};
										5'd1: OREG_RAM_D <= YEAR[15:8];
										5'd2: OREG_RAM_D <= YEAR[7:0];
										5'd3: OREG_RAM_D <= {DAY,MONTH};
										5'd4: OREG_RAM_D <= DAYS;
										5'd5: OREG_RAM_D <= HOUR;
										5'd6: OREG_RAM_D <= MIN;
										5'd7: OREG_RAM_D <= SEC;
										5'd8: OREG_RAM_D <= 8'h00;
										5'd9: OREG_RAM_D <= {4'b0000,AC};
										5'd10: OREG_RAM_D <= {1'b0,DOTSEL,2'b11,~MSHNMI_N,1'b1,~SYSRES_N,~SNDRES_N};
										5'd11: OREG_RAM_D <= {1'b0,~CDRES_N,6'b000000};
										5'd12: OREG_RAM_D <= SMEM_Q;
										5'd13: OREG_RAM_D <= SMEM_Q;
										5'd14: OREG_RAM_D <= SMEM_Q;
										5'd15: OREG_RAM_D <= SMEM_Q;
										5'd31: OREG_RAM_D <= COMREG;
										default:OREG_RAM_D <= 8'h00;
									endcase
									OREG_RAM_WE <= 1;
									
									if (OREG_CNT == 5'd31) begin
										SR[7:6] <= 2'b01;
										SR[5] <= 0;
										SR[3:0] <= 4'b1111;
										if (IREG[1][3]) begin
											INTBACK_EXEC <= 1;
											INTBACK_OPTIM_EN <= ~IREG[1][1];
											SR[5] <= 1;
											CHECK_CONTINUE <= 1;
										end
										CONT_PREV <= 0;
										MIRQ_N <= 0;
										COMM_ST <= CS_END;
									end
									OREG_CNT <= OREG_CNT + 5'd1;
								end else begin
									COMM_ST <= CS_END;
								end
							end
							
							8'h16: begin		//SETTIME
								STE <= 1;
								COMM_ST <= CS_END;
							end
							
							8'h17: begin		//SETSMEM
								OREG_CNT <= OREG_CNT + 5'd1;
								if (OREG_CNT == 5'd3) begin
									COMM_ST <= CS_END;
								end
							end
							
							8'h18: begin		//NMIREQ
								MSHNMI_N <= 0;
								COMM_ST <= CS_END;
							end
							
							8'h19: begin		//RESENAB
								RESD <= 0;
								COMM_ST <= CS_END;
							end
							
							8'h1A: begin		//RESDISA
								RESD <= 1;
								COMM_ST <= CS_END;
							end
							
							default: begin
								COMM_ST <= CS_END;
							end
						endcase
					end
					
					CS_INTBACK_PERI: begin
						if (snac && OREG_CNT != 5'd31) begin						
							if (OREG_CNT < port2Start) begin//port1(snac)
								INPUT_ACT <= 0;
								OREG_RAM_D <= UserData[OREG_CNT];							
							end else begin//port2(usb1)
								if (INPUT_ACT && INPUT_WE) begin
									OREG_RAM_D <= INPUT_DATA;
								end							
							end
							OREG_CNT <= OREG_CNT + 5'd1;				
							OREG_RAM_WA <= OREG_CNT;
							OREG_RAM_WE <= 1;
							if (OREG_CNT == port2Start - 2 || OREG_CNT == port2Start - 1 ) INPUT_ACT <= 1;
							if (OREG_CNT == 5'd30) INPUT_ACT <= 0;
							if ((OREG_CNT > port2Start + DataLUT[JOY1_TYPE]-1) && OREG_CNT < 5'd31 ) OREG_RAM_D <= 8'h00;//real HW doesn't do this					
						end else if (~snac && OREG_CNT != 5'd31) begin
							if (INPUT_ACT && INPUT_WE) begin
								OREG_CNT <= OREG_CNT + 5'd1;
								if (OREG_CNT == 5'd30) begin
									INPUT_ACT <= 0;
								end
								OREG_RAM_WA <= OREG_CNT;
								OREG_RAM_D <= INPUT_DATA;
								OREG_RAM_WE <= 1;
							end
						end else if (OREG_CNT == 5'd31) begin
							OREG_CNT <= OREG_CNT + 5'd1;
							OREG_RAM_WA <= OREG_CNT;
							OREG_RAM_D <= COMREG;
							OREG_RAM_WE <= 1;
							SR[7:5] <= {1'b1,1'b1,1'b0};
							//CHECK_CONTINUE <= 1;//TODO: multiple requests for large peripheral data
							MIRQ_N <= 0;
							COMM_ST <= CS_INTBACK_BREAK;
						end
					end
					
					CS_INTBACK_BREAK: begin
						INTBACK_EXEC <= 0;
						INTBACK_PERI <= 0;
						SF <= 0;
						COMM_ST <= CS_IDLE;
					end
					
					CS_END: begin
						case (COMREG) 
							8'h00: begin		//MSHON
								
							end
							
							8'h02: begin		//SSHON
								
							end
							
							8'h03: begin		//SSHOFF
								
							end
							
							8'h06: begin		//SNDON
								
							end
							
							8'h07: begin		//SNDOFF
								
							end
							
							8'h08: begin		//CDON
								
							end
							
							8'h09: begin		//CDOFF
								
							end
							
							8'h0D: begin		//SYSRES
								MSHRES_N <= 1;
								MSHNMI_N <= 1;
								SSHRES_N <= 1;
								SSHNMI_N <= 1;
							end
							
							8'h0E: begin		//CKCHG352
								MSHNMI_N <= 1;
							end
							
							8'h0F: begin		//CKCHG320
								MSHNMI_N <= 1;
							end
							
							8'h10: begin		//INTBACK

							end
							
							8'h16: begin		//SETTIME
								
							end
							
							8'h17: begin		//SETSMEM
								
							end
							
							8'h18: begin		//NMIREQ
								MSHNMI_N <= 1;
							end
							
							8'h19: begin		//RESENAB
								
							end
							
							8'h1A: begin		//RESDISA
								
							end
							
							default:;
						endcase
						SF <= 0;
						COMM_ST <= CS_IDLE;
					end
				endcase
				
				if (!IRQV_N && IRQV_N_OLD) begin
					VBLANK_PEND <= 1;
				end
			end
			
			RW_N_OLD <= RW_N;
			if (!RW_N && RW_N_OLD && !CS_N) begin
				case ({A,1'b1})
					7'h01: begin 
						{CONT,BREAK} <= DI[7:6];
						IREG[0] <= DI;
					end
					7'h03: IREG[1] <= DI;
					7'h05: IREG[2] <= DI;
					7'h07: IREG[3] <= DI;
					7'h09: IREG[4] <= DI;
					7'h0B: IREG[5] <= DI;
					7'h0D: IREG[6] <= DI;
					7'h1F: begin COMREG <= DI; COMREG_SET <= 1; end
					7'h63: if (DI[0]) SF <= 1;
					7'h75: {PDR1O7,PDR1O} <= DI;
					7'h77: {PDR2O7,PDR2O} <= DI;
					7'h79: DDR1 <= DI[6:0];
					7'h7B: DDR2 <= DI[6:0];
					7'h7D: IOSEL <= DI[1:0];
//					7'h7F: EXLE <= DI[1:0];
					default:;
				endcase
			end 
			
			CS_N_OLD <= CS_N;
			if (!CS_N && CS_N_OLD && RW_N) begin
				if ({A,1'b1} <= 7'h5F)
					REG_DO <= OREG_RAM_Q;
				else
					case ({A,1'b1})
						7'h61: REG_DO <= SR;
						7'h63: REG_DO <= {7'b0000000,SF};
						//7'h75: REG_DO <= {PDR1O7,PDR1I};
						7'h75: REG_DO <= {PDR1O7,snac ? USERJOYSTICK : PDR1I};
						7'h77: REG_DO <= {PDR2O7,PDR2I};
						//7'h77: REG_DO <= {PDR2O7,(snac && joyswap) ? USERJOYSTICK : PDR2I};
						default: REG_DO <= '0;
					endcase
			end
		end
	end
	
	bit [ 7: 0] SMEM_Q;
	SMPC_SMEM SMEM (CLK, OREG_CNT[1:0], IREG[OREG_CNT[2:0]], (COMM_ST == CS_EXEC && COMREG == 8'h17 && CE), OREG_CNT[1:0], SMEM_Q);
	
	bit [4:0] OREG_RAM_WA;
	bit [7:0] OREG_RAM_D;
	bit       OREG_RAM_WE;
	bit [7:0] OREG_RAM_Q;
	SMPC_OREG_RAM OREG_RAM (CLK, OREG_RAM_WA, OREG_RAM_D, OREG_RAM_WE, (A - 6'h10), OREG_RAM_Q);
	
	assign INPUT_POS = OREG_CNT;
	
	assign DO = REG_DO;

endmodule


module SMPC_OREG_RAM
(
	input        CLK,
	input  [4:0] WADDR,
	input  [7:0] DATA,
	input        WREN,
	input  [4:0] RADDR,
	output [7:0] Q
);

	wire [7:0] sub_wire0;
	
	altdpram	altdpram_component (
				.data (DATA),
				.inclock (CLK),
				.rdaddress (RADDR),
				.wraddress (WADDR),
				.wren (WREN),
				.q (sub_wire0),
				.aclr (1'b0),
				.byteena (1'b1),
				.inclocken (1'b1),
				.rdaddressstall (1'b0),
				.rden (1'b1),
//				.sclr (1'b0),
				.wraddressstall (1'b0));
	defparam
		altdpram_component.indata_aclr = "OFF",
		altdpram_component.indata_reg = "INCLOCK",
		altdpram_component.intended_device_family = "Cyclone V",
		altdpram_component.lpm_type = "altdpram",
		altdpram_component.outdata_aclr = "OFF",
		altdpram_component.outdata_reg = "UNREGISTERED",
		altdpram_component.ram_block_type = "MLAB",
		altdpram_component.rdaddress_aclr = "OFF",
		altdpram_component.rdaddress_reg = "UNREGISTERED",
		altdpram_component.rdcontrol_aclr = "OFF",
		altdpram_component.rdcontrol_reg = "UNREGISTERED",
		altdpram_component.read_during_write_mode_mixed_ports = "CONSTRAINED_DONT_CARE",
		altdpram_component.width = 8,
		altdpram_component.widthad = 5,
		altdpram_component.width_byteena = 1,
		altdpram_component.wraddress_aclr = "OFF",
		altdpram_component.wraddress_reg = "INCLOCK",
		altdpram_component.wrcontrol_aclr = "OFF",
		altdpram_component.wrcontrol_reg = "INCLOCK";
		
	assign Q = sub_wire0;
	
endmodule


module SMPC_SMEM
(
	input        CLK,
	input  [1:0] WADDR,
	input  [7:0] DATA,
	input        WREN,
	input  [1:0] RADDR,
	output [7:0] Q
);

	wire [7:0] sub_wire0;
	
	altdpram	altdpram_component (
				.data (DATA),
				.inclock (CLK),
				.rdaddress (RADDR),
				.wraddress (WADDR),
				.wren (WREN),
				.q (sub_wire0),
				.aclr (1'b0),
				.byteena (1'b1),
				.inclocken (1'b1),
				.rdaddressstall (1'b0),
				.rden (1'b1),
//				.sclr (1'b0),
				.wraddressstall (1'b0));
	defparam
		altdpram_component.indata_aclr = "OFF",
		altdpram_component.indata_reg = "INCLOCK",
		altdpram_component.intended_device_family = "Cyclone V",
		altdpram_component.lpm_type = "altdpram",
		altdpram_component.outdata_aclr = "OFF",
		altdpram_component.outdata_reg = "UNREGISTERED",
		altdpram_component.ram_block_type = "MLAB",
		altdpram_component.rdaddress_aclr = "OFF",
		altdpram_component.rdaddress_reg = "UNREGISTERED",
		altdpram_component.rdcontrol_aclr = "OFF",
		altdpram_component.rdcontrol_reg = "UNREGISTERED",
		altdpram_component.read_during_write_mode_mixed_ports = "CONSTRAINED_DONT_CARE",
		altdpram_component.width = 8,
		altdpram_component.widthad = 2,
		altdpram_component.width_byteena = 1,
		altdpram_component.wraddress_aclr = "OFF",
		altdpram_component.wraddress_reg = "INCLOCK",
		altdpram_component.wrcontrol_aclr = "OFF",
		altdpram_component.wrcontrol_reg = "INCLOCK";
		
	assign Q = sub_wire0;
	
endmodule
