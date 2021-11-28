/*
 *  Bounding Box Sample Test Iteration
 *
 *  Inputs:
 *    BBox and triangle Information
 *
 *  Outputs:
 *    Subsample location and triangle Information
 *
 *  Function:
 *    Iterate from left to right bottom to top
 *    across the bounding box.
 *
 *    While iterating set the halt signal in
 *    order to hold the bounding box pipeline in
 *    place.
 *
 *
 * Long Description:
 *    The iterator starts in the waiting state,
 *    when a valid triangle bounding box
 *    appears at the input. It will enter the
 *    testing state the next cycle with a
 *    sample equivelant to the lower left
 *    cooridinate of the bounding box.
 *
 *    While in the testing state, the next sample
 *    for each cycle should be one sample interval
 *    to the right, except when the current sample
 *    is at the right edge.  If the current sample
 *    is at the right edge, the next sample should
 *    be one row up.  Additionally, if the current
 *    sample is on the top row and the right edge,
 *    next cycles sample should be invalid and
 *    equivelant to the lower left vertice and
 *    next cycles state should be waiting.
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Tue 10/01/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 *
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 19, 2012   jingpu    ported from John's original code to Genesis
 *
 * ***************************************************************************/

/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxN
 * where R indicates that it is a Raster Block signal
 * xx indicates the clock slice that it belongs to
 * and N indicates the type of signal that it is.
 * H indicates logic high, L indicates logic low,
 * U indicates unsigned fixed point, and S indicates
 * signed fixed point.
 *
 * For all the signed fixed point signals (logic signed [`$sig_fig`-1:0]),
 * their highest `$sig_fig-$radix` bits, namely [`$sig_fig-1`:`$radix`]
 * represent the integer part of the fixed point number,
 * while the lowest `$radix` bits, namely [`$radix-1`:0]
 * represent the fractional part of the fixed point number.
 *
 *
 *
 * For signal subSample_RnnnnU (logic [3:0])
 * 1000 for  1x MSAA eq to 1 sample per pixel
 * 0100 for  4x MSAA eq to 4 samples per pixel,
 *              a sample is half a pixel on a side
 * 0010 for 16x MSAA eq to 16 sample per pixel,
 *              a sample is a quarter pixel on a side.
 * 0001 for 64x MSAA eq to 64 samples per pixel,
 *              a sample is an eighth of a pixel on a side.
 *
 */

module test_iterator
#(
    parameter SIGFIG = 24, // Bits in color and position.
    parameter RADIX = 10, // Fraction bits in color and position
    parameter VERTS = 3, // Maximum Vertices in triangle
    parameter AXIS = 3, // Number of axis foreach vertex 3 is (x,y,z).
    parameter COLORS = 3, // Number of color channels
    parameter PIPE_DEPTH = 1, // How many pipe stages are in this block
    parameter MOD_FSM = 0 // Use Modified FSM to eliminate a wait state
)
(
    //Input Signals
    input logic signed [SIGFIG-1:0]     tri_R13S[VERTS-1:0][AXIS-1:0], //triangle to Iterate Over
    input logic unsigned [SIGFIG-1:0]   color_R13U[COLORS-1:0] , //Color of triangle
    input logic signed [SIGFIG-1:0]     box_R13S[1:0][1:0], //Box to iterate for subsamples
    input logic                             validTri_R13H, //triangle is valid

    //Control Signals
    input logic [3:0]   subSample_RnnnnU , //Subsample width
    output logic        halt_RnnnnL , //Halt -> hold current microtriangle
    //Note that this block generates
    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset
    
    input logic hit_valid, //  Signal from sample test that determines if we should stop our current right/left iterating
    
    input logic signed [SIGFIG-1:0]    start_coord[VERTS-1:0],

    //Outputs
    output logic signed [SIGFIG-1:0]    tri_R14S[VERTS-1:0][AXIS-1:0], //triangle to Sample Test
    output logic unsigned [SIGFIG-1:0]  color_R14U[COLORS-1:0] , //Color of triangle
    output logic signed [SIGFIG-1:0]    sample_R14S[1:0][3:0], //Sample Location to Be Tested
    output logic                            validSamp_R14H[3:0] //Sample and triangle are Valid
);

    // This module implement a Moore machine to iterarte sample points in bbox
    // Recall: a Moore machine is an FSM whose output values are determined
    // solely by its current state.
    // A simple way to build a Moore machine is to make states for every output
    // and the values of the current states are the outputs themselves

    // Now we create the signals for the next states of each outputs and
    // then instantiate registers for storing these states
    logic signed [SIGFIG-1:0]       next_tri_R14S[VERTS-1:0][AXIS-1:0];
    logic unsigned  [SIGFIG-1:0]    next_color_R14U[COLORS-1:0] ;
    logic signed [SIGFIG-1:0]       next_sample_R14S[1:0][3:0];
    logic                               next_validSamp_R14H;
    logic                               next_halt_RnnnnL;

    // Instantiate registers for storing these states
    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d301
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_tri_R14S  ),
        .out    (tri_R14S       )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d302
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_color_R14U),
        .out    (color_R14U     )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d303
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (next_sample_R14S   ),
        .out    (sample_R14S        )
    );

    dff #(
        .WIDTH(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d304
    (
        .clk    (clk                                    ),
        .reset  (rst                                    ),
        .en     (1'b1                                   ),
        .in     ({next_validSamp_R14H, next_halt_RnnnnL}),
        .out    ({validSamp_R14H, halt_RnnnnL}          )
    );
    // Instantiate registers for storing these states

    typedef enum logic {
                            WAIT_STATE,
                            TEST_STATE
                        } state_t;
generate
if(MOD_FSM == 0) begin // Using baseline FSM
    //////
    //////  RTL code for original FSM Goes Here
    //////

    // To build this FSM we want to have two more state: one is the working
    // status of this FSM, and the other is the current bounding box where
    // we iterate sample points

    // define two more states, box_R14S and state_R14H
    logic signed [SIGFIG-1:0]   box_R14S[1:0][1:0];         // the state for current bounding box
    logic signed [SIGFIG-1:0]   next_box_R14S[1:0][1:0];

    state_t                     state_R14H;     //State Designation (Waiting or Testing)
    state_t                     next_state_R14H;        //Next Cycles State

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d305
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_box_R14S  ),
        .out    (box_R14S       )
    );

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            state_R14H <= WAIT_STATE;
        end
        else begin
            state_R14H <= next_state_R14H;
        end
    end

    // define some helper signals
    logic signed [SIGFIG-1:0]   next_up_samp_R14S[1:0][3:0]; //If jump up, next sample
    logic signed [SIGFIG-1:0]   next_rt_samp_R14S[1:0][3:0]; //If jump right, next sample
    logic                       at_right_edg_R14H;      //Current sample at right edge of bbox?
    logic                       at_top_edg_R14H;        //Current sample at top edge of bbox?
    logic                       at_end_box_R14H;        //Current sample at end of bbox?

    //////
    ////// First calculate the values of the helper signals using CURRENT STATES
    //////

    // check the comments 'A Note on Signal Names'
    // at the begining of the module for the help on
    // understanding the signals here

//    at_right_edg_R14H = 1'b0;
//    at_top_edg_R14H = 1'b0;
//    at_end_box_R14H = 1'b0;
 always_comb begin
        // START CODE HERE

         //Set next right samples
     
       for (int i=0; i<4 ; i++) begin 
           next_rt_samp_R14S[0][i] = sample_R14S[0][i]  + (subSample_RnnnnU << RADIX-3);
           next_rt_samp_R14S[1][i] = sample_R14S[1][i];
                                                 
                                                 
           next_up_samp_R14S[0][i] =  box_R14S[0][0]; //Back to min x coord
           //increment by 4
           next_up_samp_R14S[1][i] = sample_R14S[1][i] + (subSample_RnnnnU << RADIX-3) + (subSample_RnnnnU << RADIX-3) + (subSample_RnnnnU << RADIX-3) + (subSample_RnnnnU << RADIX-3);                                       
           
       end

        //only need to check one sample
        if (sample_R14S[0][0] >= box_R14S[1][0]) begin
            at_right_edg_R14H = 1'b1;

        end
        else begin
             at_right_edg_R14H = 1'b0;
        end

        //only need to check one
       if (sample_R14S[1][3] >= box_R14S[1][1]) begin
            at_top_edg_R14H = 1'b1;

        end
        else begin
             at_top_edg_R14H = 1'b0;
        end

        if (at_right_edg_R14H && at_top_edg_R14H) begin
             at_end_box_R14H = 1'b1;
        end
        else begin
             at_end_box_R14H = 1'b0;
        end




        // END CODE HERE
    end

    //////
    ////// Then complete the following combinational logic defining the
    ////// next states
    //////

    ////// COMPLETE THE FOLLOW ALWAYS_COMB BLOCK

    // Combinational logic for state transitions
    always_comb begin
        // START CODE HERE

        case(state_R14H) 

            1'b0: begin

                //If we are in waiting, and we see a valid signal, we switch to testing
                // Otherwise we stay in waiting
                if (validTri_R13H) begin
                    next_state_R14H = TEST_STATE;

                    //Once we see a valid, we will halt the bounding box above
                    next_halt_RnnnnL = 1'b0;

                    //Set current bbox to input bounding box:
                    next_box_R14S = box_R13S;

                    // Next sample is valid
                    next_validSamp_R14H[0] = 1'b1;
                    next_validSamp_R14H[1] = 1'b1;
                    next_validSamp_R14H[2] = 1'b1;
                    next_validSamp_R14H[3] = 1'b1;
                    
                  
                    

                    //Next sample is lower left vertex
                    next_sample_R14S[0][0] = box_R13S[0][0];
                    next_sample_R14S[1][0] = box_R13S[0][1];
                    
                    next_sample_R14S[0][1] = box_R13S[0][0];
                    next_sample_R14S[1][1] = box_R13S[0][1] + (subSample_RnnnnU << RADIX-3);
                     
                    next_sample_R14S[0][2] = box_R13S[0][0];
                    next_sample_R14S[1][2] = box_R13S[0][1] + (subSample_RnnnnU << RADIX-3)  + (subSample_RnnnnU << RADIX-3);
                                      
                    next_sample_R14S[0][3] = box_R13S[0][0];
                    next_sample_R14S[1][3] = box_R13S[0][1] + (subSample_RnnnnU << RADIX-3)  + (subSample_RnnnnU << RADIX-3) + (subSample_RnnnnU << RADIX-3);
                    

                    // Set current tri to input tri
                    next_tri_R14S = tri_R13S;

                    // Set current to input
                    next_color_R14U = color_R13U;

                end
                else begin
                    next_state_R14H = WAIT_STATE;

                    // We keep waiting, and do not hold the pipline above
                    next_halt_RnnnnL = 1'b1;

                    next_box_R14S = box_R13S;

                    // Next sample is invalid
                    next_validSamp_R14H[0] = 1'b0;
                    next_validSamp_R14H[1] = 1'b0;
                    next_validSamp_R14H[2] = 1'b0;
                    next_validSamp_R14H[3] = 1'b0;

                    //Next sample is lower left vertex
                    next_sample_R14S[0][0] = box_R13S[0][0];
                    next_sample_R14S[1][0] = box_R13S[0][1];
                    
                    next_sample_R14S[0][1] = box_R13S[0][0];
                    next_sample_R14S[1][1] = box_R13S[0][1] + (subSample_RnnnnU << RADIX-3);
                     
                    next_sample_R14S[0][2] = box_R13S[0][0];
                    next_sample_R14S[1][2] = box_R13S[0][1] + (subSample_RnnnnU << RADIX-3)  + (subSample_RnnnnU << RADIX-3);
                                      
                    next_sample_R14S[0][3] = box_R13S[0][0];
                    next_sample_R14S[1][3] = box_R13S[0][1] + (subSample_RnnnnU << RADIX-3)  + (subSample_RnnnnU << RADIX-3) + (subSample_RnnnnU << RADIX-3);


                    // Set current tri to input tri
                    next_tri_R14S = tri_R13S;

                     // Set current to input
                    next_color_R14U = color_R13U;
                    
                end

            end

            1'b1: begin
                
                //If we are in testing, and reach the top right corner, we move to waiting
                // Our next sample is invalid
                // And we no longer halt
                // Next sample is LL vertex
                if (at_end_box_R14H) begin
                    next_state_R14H = WAIT_STATE;

                    // No longer halt -> iterating is over
                    next_halt_RnnnnL = 1'b1;

                     // Preserve bounding box (will be reset in next state)
                    next_box_R14S = box_R14S;

                    // Next sample is invalid
                    next_validSamp_R14H[0] = 1'b0;
                    next_validSamp_R14H[1] = 1'b0;
                    next_validSamp_R14H[2] = 1'b0;
                    next_validSamp_R14H[3] = 1'b0;


             //Next sample is lower left vertex
                    next_sample_R14S[0][0] = box_R14S[0][0];
                    next_sample_R14S[1][0] = box_R14S[0][1];
                    
                    next_sample_R14S[0][1] = box_R14S[0][0];
                    next_sample_R14S[1][1] = box_R14S[0][1] + (subSample_RnnnnU << RADIX-3);
                     
                    next_sample_R14S[0][2] = box_R14S[0][0];
                    next_sample_R14S[1][2] = box_R14S[0][1] + (subSample_RnnnnU << RADIX-3)  + (subSample_RnnnnU << RADIX-3);
                                      
                    next_sample_R14S[0][3] = box_R14S[0][0];
                    next_sample_R14S[1][3] = box_R14S[0][1] + (subSample_RnnnnU << RADIX-3)  + (subSample_RnnnnU << RADIX-3) + (subSample_RnnnnU << RADIX-3);


                    //Hold
                    next_tri_R14S = tri_R14S;

                     // Set current to input
                    next_color_R14U = color_R14U;
                    

                end
                else begin

                    // We remain in testing
                    next_state_R14H = TEST_STATE;

                    //Since we are iterating, the halt signal is 0 (ie we halt bbox)
                    next_halt_RnnnnL = 1'b0;


                    // Preserve current bounding box
                    next_box_R14S = box_R14S;

                    // Next sample might be valid. See if current sample is on edge. if it is, not valid
                    //Note that some samples could be valid while others are not!!
                    for (int z =0; z < 4; z++) begin
                        if (sample_R14S[0][z] >= box_R14S[1][0] || sample_R14S[1][z] >= box_R14S[1][1]) begin
                            next_validSamp_R14H[z] = 1'b0;
                            
                        end
                        else begin
                            next_validSamp_R14H[z] = 1'b1;
                        end
                        
                    end
                    

                    // Hold
                    next_tri_R14S = tri_R14S;


                    next_color_R14U = color_R14U;


                    // Set next sample
                    
                    if (at_right_edg_R14H) begin
                        // If we are at the right edge we get the next up sample
                        next_sample_R14S = next_up_samp_R14S;
                    end
                    else begin
                        // If we are NOT at the right edge we get the next right sample
                         next_sample_R14S = next_rt_samp_R14S;
                        
                    end

                end

            end
      
        endcase




        // Try using a case statement on state_R14H
        // END CODE HERE
    end // always_comb

    //Assertions for testing FSM logic

    // Write assertions to verify your FSM transition sequence
    // Can you verify that:
    // 1) A validTri_R13H signal causes a transition from WAIT state to TEST state
    
    // 2) An end_box_R14H signal causes a transition from TEST state to WAIT state

   
    // 3) What are you missing?

    //Your assertions goes here
    // START CODE HERE
        
     // 1) 
        
     assert property(@(posedge clk) ((state_R14H == WAIT_STATE) && validTri_R13H ) |-> (next_state_R14H == TEST_STATE) );

     // 2)
     assert property(@(posedge clk) ((state_R14H == TEST_STATE) && at_end_box_R14H ) |-> (next_state_R14H == WAIT_STATE) );

        
    // 3)
    // IN wait state, non valid input box, stay in wait state
    assert property(@(posedge clk) ((state_R14H == WAIT_STATE) && !validTri_R13H ) |-> (next_state_R14H == WAIT_STATE) );

    // In testing state, not at top right, stay in test state
    assert property(@(posedge clk) ((state_R14H == TEST_STATE) && !at_end_box_R14H ) |-> (next_state_R14H == TEST_STATE) );

    // END CODE HERE
    // Assertion ends

    //////
    //////  RTL code for original FSM Finishes
    //////

    //Some Error Checking Assertions

    //Define a Less Than Property
    //
    //  a should be less than b
    property rb_lt( rst, a , b , c );
        @(posedge clk) rst | ((a<=b) | !c);
    endproperty

    //Check that Proposed Sample is in BBox
    // START CODE HERE

    assert property(rb_lt(rst, box_R14S[0][0], sample_R14S[0], validSamp_R14H));
    assert property(rb_lt(rst, sample_R14S[0], box_R14S[1][0],  validSamp_R14H));
    assert property(rb_lt(rst, box_R14S[0][1], sample_R14S[1], validSamp_R14H));
    assert property(rb_lt(rst, sample_R14S[1], box_R14S[1][1],  validSamp_R14H));
  // END CODE HERE
    //Check that Proposed Sample is in BBox

    //Error Checking Assertions
end 
else begin // Use modified FSM 
    
    //Better Bounding Box iterator per the paper

    //////
    //////  RTL code for modified FSM Goes Here
    //////

    ////// PLACE YOUR CODE HERE
    
      //////
    //////  RTL code for original FSM Goes Here
    //////

    // To build this FSM we want to have two more state: one is the working
    // status of this FSM, and the other is the current bounding box where
    // we iterate sample points

    // define two more states, box_R14S and state_R14H
    logic signed [SIGFIG-1:0]   box_R14S[1:0][1:0];         // the state for current bounding box
    logic signed [SIGFIG-1:0]   next_box_R14S[1:0][1:0];
    
    // definte two states to keep track of how many invalids we see
    // We must set a buffer because otherwise when we first move up a row we would immediatly fail our test:
    logic signed [3:0] invalid_counter;
    logic signed [3:0] next_invalid_counter;
    
    //define two states to keep track of the iterating direction
    logic  iterate_direction; //0 Right, 1 Left
    logic next_iterate_direction;
    
    
    //define two states to keep track of if we are in the first row
    logic  first_row; //1 true
    logic next_first_row; //0 false
    
    
//     logic signed [SIGFIG-1:0]   start[VERTS-1:0];
//     logic signed [SIGFIG-1:0]   next_start[VERTS-1:0];
    

    state_t                     state_R14H;     //State Designation (Waiting or Testing)
    state_t                     next_state_R14H;        //Next Cycles State

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d305
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_box_R14S  ),
        .out    (box_R14S       )
    );

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            state_R14H <= WAIT_STATE;
            invalid_counter <= 4'b0;
            iterate_direction <= 1'b0;
            first_row <= 1'b1;
            //start <= start_coord;
        end
        else begin
            state_R14H <= next_state_R14H;
            invalid_counter <= next_invalid_counter;
            iterate_direction <= next_iterate_direction;
            first_row <= next_first_row;
            //start <= next_start;
        end
    end

    // define some helper signals
    logic signed [SIGFIG-1:0]   next_up_samp_R14S[1:0]; //If jump up, next sample
    logic signed [SIGFIG-1:0]   next_rt_samp_R14S[1:0]; //If jump right, next sample
    logic signed [SIGFIG-1:0]   next_lt_samp_R14S[1:0]; //If jump left, next sample
    logic                       at_right_edg_R14H;      //Current sample at right edge of bbox?
    logic                       at_left_edg_R14H;      //Current sample at right edge of bbox?
    logic                       at_top_edg_R14H;        //Current sample at top edge of bbox?
    logic                       at_end_box_R14H;        //Current sample at end of bbox?

    
   
    

    //////
    ////// First calculate the values of the helper signals using CURRENT STATES
    //////

    // check the comments 'A Note on Signal Names'
    // at the begining of the module for the help on
    // understanding the signals here

//    at_right_edg_R14H = 1'b0;
//    at_top_edg_R14H = 1'b0;
//    at_end_box_R14H = 1'b0;
 always_comb begin
        // START CODE HERE
    

         //Set next right sample

        //Set x coordinate of new next sample
        next_rt_samp_R14S[0] = sample_R14S[0]  + (subSample_RnnnnU << RADIX-3);
        next_lt_samp_R14S[0] = sample_R14S[0]  - (subSample_RnnnnU << RADIX-3);
     
        //if we are moving right
        if (!iterate_direction) begin
            next_up_samp_R14S[0] = box_R14S[1][0];
        end
        else begin
            next_up_samp_R14S[0] = box_R14S[0][0];
        end
     
        //next_up_samp_R14S[0] = sample_R14S[0]; //just move up a row keep x coord the same. TODO: investigage if padding is necessary here
     
        //set y coordinate of new next sample;
        next_rt_samp_R14S[1] = sample_R14S[1];
        next_lt_samp_R14S[1] = sample_R14S[1];
        next_up_samp_R14S[1] = sample_R14S[1] + (subSample_RnnnnU << RADIX-3);
     
     

     
     
//      case(hit_valid) 
//          1'b1: next_invalid_counter = 4'b0000;
//          1'b0: begin
             
//              if (at_right_edg_R14H || at_left_edg_R14H) begin
//                  next_invalid_counter = 4'b0000;
//              end
//              else begin
//                  next_invalid_counter = invalid_counter + 4'b0001;
//              end
//          end
//          default: next_invalid_counter = 4'b0000;
//      endcase
         
     
   
     //If we are iterating right and we aren't outside our triangle our bounding box)
     if (!iterate_direction) begin
         
         if ( (sample_R14S[0] >= box_R14S[1][0])) begin  //|| ((!hit_valid) && (invalid_counter > 4'b1111)) ) begin
              at_right_edg_R14H = 1'b1;
              at_left_edg_R14H = 1'b0;
         end
         else begin
             at_right_edg_R14H = 1'b0;
              at_left_edg_R14H = 1'b0;
         end
         
     end
     
     //If we are iterating left and we aren't outside our triangle our bounding box)
     else begin
         
         if ( (sample_R14S[0] <= box_R14S[0][0] )) begin // || ((!hit_valid) && (invalid_counter > 4'b1111))  ) begin
            at_left_edg_R14H = 1'b1;
            at_right_edg_R14H = 1'b0;
         
         end
         else begin
              at_left_edg_R14H = 1'b0;
              at_right_edg_R14H = 1'b0;
          end
     end
         
         
         
     
      // Check top 
      if (sample_R14S[1] >= box_R14S[1][1]) begin
            at_top_edg_R14H = 1'b1;

        end
        else begin
             at_top_edg_R14H = 1'b0;
 
        end
        
         //if we at the top and we finish iterating either to the left or the right, we are done
        if ((at_right_edg_R14H || at_left_edg_R14H)  && at_top_edg_R14H) begin
             at_end_box_R14H = 1'b1;
        end
        else begin
             at_end_box_R14H = 1'b0;
        end
     
     





        // END CODE HERE
    end

    //////
    ////// Then complete the following combinational logic defining the
    ////// next states
    //////

    ////// COMPLETE THE FOLLOW ALWAYS_COMB BLOCK

    // Combinational logic for state transitions
    always_comb begin
        // START CODE HERE
        
         //update invalid counter:
         if (at_right_edg_R14H || at_left_edg_R14H || hit_valid) begin
              next_invalid_counter = 4'b0;
         end
         else begin
             next_invalid_counter = invalid_counter + 4'b0001;
         end

        case(state_R14H) 

            1'b0: begin

                //If we are in waiting, and we see a valid signal, we switch to testing
                // Otherwise we stay in waiting
                if (validTri_R13H) begin
                    next_state_R14H = TEST_STATE;

                    //Once we see a valid, we will halt the bounding box above
                    next_halt_RnnnnL = 1'b0;

                    //Set current bbox to input bounding box:
                    next_box_R14S = box_R13S;

                    // Next sample is valid
                    next_validSamp_R14H = 1'b1;
                    
                    //Start by iterating to the right:
                    next_iterate_direction = 1'b0;
                    

                    next_sample_R14S[0] =  start_coord[0];//tri_R13S[0][0]  - (subSample_RnnnnU << RADIX-3);
                    next_sample_R14S[1] = box_R13S[1];// + (subSample_RnnnnU << RADIX-3);        
                    //Next sample is lower left vertex  //UPDATE: Start at lowest vertex of triangle
                    //find lowest vertex (compare y coords)
                    // Not we also shift over the the LEFT a tad just so we don't miss anything
//                     if ((tri_R13S[0][1] < tri_R13S[1][1]) && (tri_R13S[0][1] < tri_R13S[2][1])) begin      
//                         next_sample_R14S[0] = start_coord[0];//  - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R13S[1];// + (subSample_RnnnnU << RADIX-3);        
//                     end
                    
//                     else if ((tri_R13S[1][1] < tri_R13S[0][1]) && (tri_R13S[1][1] < tri_R13S[2][1])) begin
//                         next_sample_R14S[0] = box_R13S[0][0];// - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R13S[0][1];// + (subSample_RnnnnU << RADIX-3);  
//                     end
                    
//                     else begin
//                         next_sample_R14S[0] = box_R13S[0][0];//  - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R13S[0][1];// + (subSample_RnnnnU << RADIX-3);
//                     end


                    // Set current tri to input tri
                    next_tri_R14S = tri_R13S;

                    // Set current to input
                    next_color_R14U = color_R13U;
                    
                    // Set next first row to true
                    next_first_row = 1'b1;

        
                end
                else begin
                    next_state_R14H = WAIT_STATE;

                    // We keep waiting, and do not hold the pipline above
                    next_halt_RnnnnL = 1'b1;

                    next_box_R14S = box_R13S;

                    // Next sample is invalid
                    next_validSamp_R14H = 1'b0;
                    
                    //Start by iterating to the right:
                    next_iterate_direction = 1'b0;

                    
                    next_sample_R14S[0] =  start_coord[0];//tri_R13S[0][0]  - (subSample_RnnnnU << RADIX-3);
                    next_sample_R14S[1] = box_R13S[1];// + (subSample_RnnnnU << RADIX-3);        
                    //Next sample is lower left vertex  //UPDATE: Start at lowest vertex of triangle
                    //find lowest vertex (compare y coords)
                    // Not we also shift over the the LEFT a tad just so we don't miss anything
//                     if ((tri_R13S[0][1] < tri_R13S[1][1]) && (tri_R13S[0][1] < tri_R13S[2][1])) begin      
//                         next_sample_R14S[0] = box_R13S[0][0];// - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R13S[0][1];// + (subSample_RnnnnU << RADIX-3);        
//                     end
                    
//                     else if ((tri_R13S[1][1] < tri_R13S[0][1]) && (tri_R13S[1][1] < tri_R13S[2][1])) begin
//                         next_sample_R14S[0] = box_R13S[0][0];//  - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R13S[0][1];// + (subSample_RnnnnU << RADIX-3);  
//                     end
                    
//                     else begin
//                         next_sample_R14S[0] = box_R13S[0][0];// - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R13S[0][1];// + (subSample_RnnnnU << RADIX-3);
//                     end
                    

                    // Set current tri to input tri
                    next_tri_R14S = tri_R13S;

                     // Set current to input
                    next_color_R14U = color_R13U;
                    
                    
                    // Set next first row to true
                    next_first_row = 1'b1;

                    
                end

            end

            1'b1: begin
                
                //If we are in testing, and reach the top right corner, we move to waiting
                // Our next sample is invalid
                // And we no longer halt
                // Next sample is LL vertex
                if (at_end_box_R14H) begin
                    next_state_R14H = WAIT_STATE;

                    // No longer halt -> iterating is over
                    next_halt_RnnnnL = 1'b1;

                     // Preserve bounding box (will be reset in next state)
                    next_box_R14S = box_R14S;

                    // Next sample is invalid
                    next_validSamp_R14H = 1'b0;
                    
                    //Start by iterating to the right:
                    next_iterate_direction = 1'b0;


                    next_sample_R14S[0] = start_coord[0];//tri_R13S[0][0]  - (subSample_RnnnnU << RADIX-3);
                    next_sample_R14S[1] = box_R14S[1];// + (subSample_RnnnnU << RADIX-3);              
                    //Next sample is lower left vertex  //UPDATE: Start at lowest vertex of triangle
                    //find lowest vertex (compare y coords)
                    // Not we also shift over the the LEFT a tad just so we don't miss anything
//                     if ((tri_R14S[0][1] < tri_R14S[1][1]) && (tri_R14S[0][1] < tri_R14S[2][1])) begin      
//                         next_sample_R14S[0] = box_R14S[0][0]; // - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R14S[0][1];// + (subSample_RnnnnU << RADIX-3);        
//                     end
                    
//                     else if ((tri_R14S[1][1] < tri_R14S[0][1]) && (tri_R14S[1][1] < tri_R14S[2][1])) begin
//                         next_sample_R14S[0] = box_R14S[0][0];//  - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R14S[0][1];// + (subSample_RnnnnU << RADIX-3);  
//                     end
                    
//                     else begin
//                         next_sample_R14S[0] = box_R14S[0][0];// - (subSample_RnnnnU << RADIX-3);
//                         next_sample_R14S[1] = box_R14S[0][1];// + (subSample_RnnnnU << RADIX-3);
//                     end


                    //Hold
                    next_tri_R14S = tri_R14S;

                     // Set current to input
                    next_color_R14U = color_R14U;
                    
                     // Set next first row to true
                    next_first_row = 1'b1;

                    

                end
                else begin

                    // We remain in testing
                    next_state_R14H = TEST_STATE;

                    //Since we are iterating, the halt signal is 0 (ie we halt bbox)
                    next_halt_RnnnnL = 1'b0;


                    // Preserve current bounding box
                    next_box_R14S = box_R14S;

                    // Next sample is valid
                    next_validSamp_R14H = 1'b1;

                    // Hold
                    next_tri_R14S = tri_R14S;


                    next_color_R14U = color_R14U;


                    // Set next sample
                    
                    //if we are currently iterating to the right)
                    if (!iterate_direction) begin
                        
                        if (at_right_edg_R14H) begin
                            // If we are at the right edge we get the next up sample
                            next_sample_R14S = next_up_samp_R14S;
                            //Start iterating to left
                            next_iterate_direction = 1'b1;
                            
                             // Set next first row to true
                            next_first_row = 1'b0;


                        end
                        else begin
                            // If we are NOT at the right edge we get the next right sample
                             next_sample_R14S = next_rt_samp_R14S;
                            
                            //keep iterating to the right
                             next_iterate_direction = 1'b0;
                            
                             next_first_row = first_row;

                        end
                        
                    end
                    
                    //Otherwise we are iterating to the left
                    else begin
                        
                        if (at_left_edg_R14H) begin
                            // If we are at the right edge we get the next up sample
                            next_sample_R14S = next_up_samp_R14S;
                            //Start iterating to right
                            next_iterate_direction = 1'b0;
                            
                            next_first_row = 1'b0;

                        end
                        else begin
                            // If we are NOT at the left edge we get the next left sample
                             next_sample_R14S = next_lt_samp_R14S;
                            
                            //keep iterating to the left
                             next_iterate_direction = 1'b1;
                            
                             next_first_row = first_row;
                        end
                        
                    end
                        
                end

            end
      
        endcase




        // Try using a case statement on state_R14H
        // END CODE HERE
    end // always_comb

    //Assertions for testing FSM logic

    // Write assertions to verify your FSM transition sequence
    // Can you verify that:
    // 1) A validTri_R13H signal causes a transition from WAIT state to TEST state
    
    // 2) An end_box_R14H signal causes a transition from TEST state to WAIT state

   
    // 3) What are you missing?

    //Your assertions goes here
    // START CODE HERE
        
     // 1) 
        
     assert property(@(posedge clk) ((state_R14H == WAIT_STATE) && validTri_R13H ) |-> (next_state_R14H == TEST_STATE) );

     // 2)
     assert property(@(posedge clk) ((state_R14H == TEST_STATE) && at_end_box_R14H ) |-> (next_state_R14H == WAIT_STATE) );

        
    // 3)
    // IN wait state, non valid input box, stay in wait state
    assert property(@(posedge clk) ((state_R14H == WAIT_STATE) && !validTri_R13H ) |-> (next_state_R14H == WAIT_STATE) );

    // In testing state, not at top right, stay in test state
    assert property(@(posedge clk) ((state_R14H == TEST_STATE) && !at_end_box_R14H ) |-> (next_state_R14H == TEST_STATE) );

    // END CODE HERE
    // Assertion ends

    //////
    //////  RTL code for original FSM Finishes
    //////

    //Some Error Checking Assertions

    //Define a Less Than Property
    //
    //  a should be less than b
    property rb_lt( rst, a , b , c );
        @(posedge clk) rst | ((a<=b) | !c);
    endproperty

    //Check that Proposed Sample is in BBox
    // START CODE HERE

    assert property(rb_lt(rst, box_R14S[0][0], sample_R14S[0], validSamp_R14H));
    assert property(rb_lt(rst, sample_R14S[0], box_R14S[1][0],  validSamp_R14H));
    assert property(rb_lt(rst, box_R14S[0][1], sample_R14S[1], validSamp_R14H));
    assert property(rb_lt(rst, sample_R14S[1], box_R14S[1][1],  validSamp_R14H));

    //////
    //////  RTL code for modified FSM Finishes
    //////

end
endgenerate

endmodule



