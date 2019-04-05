/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */


#include "hilevel.h"

#define maxPrograms 20
#define priorityWeight 3


pcb_t pcb[ maxPrograms ]; pcb_t* current = NULL;
int n_prog = 0;

void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

    current = next;                             // update   executing index   to P_{next}

  return;
}

void schedule( ctx_t* ctx, bool terminated) {
  int priority = 0, r_priority = 0, max_priority = 0;
  int next_id, current_id;

  // find information about current process
  for (int i = 0; i<n_prog; i++){
    if(pcb[i].status == STATUS_EXECUTING){
      r_priority = pcb[i].priority * priorityWeight + pcb[i].age;
      current_id = i;
       char h = '0' + r_priority;
       PL011_putc( UART0, h, true );
       // char h = '0' + current_id;
       // PL011_putc( UART0, h, true );
    }
  }

  // change current status if it was terminated to avoid future wrong calculations
  if (terminated) current->status = STATUS_TERMINATED;

  // find the next process with the highest priority
  for (int i = 0; i<n_prog; i++){
    if(pcb[i].status != STATUS_EXECUTING
      && pcb[i].status != STATUS_WAITING
      && pcb[i].status != STATUS_TERMINATED){
        priority = pcb[i].priority * priorityWeight + pcb[i].age;
        if (priority > max_priority){
          max_priority = priority;
          next_id = i;
        }
      }
  }
  // char h = '0' + next_id;
  // PL011_putc( UART0, h, true );
   char h = '0' + max_priority;
   PL011_putc( UART0, h, true );

   if(terminated){
     // set status
     pcb[next_id].status = STATUS_EXECUTING;
     // reset age
     pcb[next_id].age = 0;
     // switch context
     dispatch(ctx, &pcb[current_id], &pcb[next_id]);
   } else {
      // context switch to the next process with the highest priority
      if(max_priority > r_priority && next_id != current_id){

        // update statuses
        pcb[current_id].status = STATUS_READY;
        pcb[next_id].status = STATUS_EXECUTING;
        // reset age of the process that is about to be executed
        pcb[next_id].age = 0;

        // switch context
        dispatch(ctx, &pcb[current_id], &pcb[next_id]);
      }
    }

  // increase the age of the rest of the processes
  for(int i = 0; i<n_prog; i++)
    if(pcb[i].status != STATUS_EXECUTING
      && pcb[i].status != STATUS_WAITING
      && pcb[i].status != STATUS_TERMINATED
      && pcb[i].pid != 0){
        pcb[i].age ++;
      }

  return;
}

extern void     main_P3();
extern uint32_t tos_P1;
extern void     main_P4();
extern uint32_t tos_P2;
extern void     main_P5();
extern uint32_t tos_P3;
extern uint32_t tos_P4;
extern uint32_t tos_P5;
extern uint32_t tos_P6;
extern uint32_t tos_P7;
extern uint32_t tos_P8;
extern uint32_t tos_P9;
extern uint32_t tos_P10;
extern uint32_t tos_P11;
extern uint32_t tos_P12;
extern uint32_t tos_P13;
extern uint32_t tos_P14;
extern uint32_t tos_P15;
extern uint32_t tos_P16;
extern uint32_t tos_P17;
extern uint32_t tos_P18;
extern uint32_t tos_P19;
extern uint32_t tos_P20;

uint32_t stacks[maxPrograms] = {};

extern void     main_console();

int getCurrentIndex(){
  int id = -1;
  for (int i=0; i< maxPrograms; i++){
    if (current->pid == pcb[i].pid){
      id = i;
      break;
    }
  }
  return id;
}

void hilevel_handler_rst( ctx_t* ctx              ) {

  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor


  memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = Console
  pcb[ 0 ].pid      = 1; // id 1 is console
  pcb[ 0 ].status   = STATUS_CREATED;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_P1  ); //empty stack
  pcb[ 0 ].priority = 2; // 1 = low, 9 = high
  pcb[ 0 ].age = 0;


  stacks[0] = ( uint32_t )( &tos_P1 );
  stacks[1] = ( uint32_t )( &tos_P2 );
  stacks[2] = ( uint32_t )( &tos_P3 );
  stacks[3] = ( uint32_t )( &tos_P4 );
  stacks[4] = ( uint32_t )( &tos_P5 );
  stacks[5] = ( uint32_t )( &tos_P6 );
  stacks[6] = ( uint32_t )( &tos_P7 );
  stacks[7] = ( uint32_t )( &tos_P8 );
  stacks[8] = ( uint32_t )( &tos_P9 );
  stacks[9] = ( uint32_t )( &tos_P10 );
  stacks[10] = ( uint32_t )( &tos_P11 );
  stacks[11] = ( uint32_t )( &tos_P12 );
  stacks[12] = ( uint32_t )( &tos_P13 );
  stacks[13] = ( uint32_t )( &tos_P14 );
  stacks[14] = ( uint32_t )( &tos_P15 );
  stacks[15] = ( uint32_t )( &tos_P16 );
  stacks[16] = ( uint32_t )( &tos_P17 );
  stacks[17] = ( uint32_t )( &tos_P18 );
  stacks[18] = ( uint32_t )( &tos_P19 );
  stacks[19] = ( uint32_t )( &tos_P20 );

  if (pcb[0].status == STATUS_CREATED){
    dispatch( ctx, NULL, &pcb[ 0 ]); // context switch to console
    pcb[0].status = STATUS_EXECUTING;
    n_prog++;
  }

  return;
}


void hilevel_handler_irq(ctx_t* ctx) {

  uint32_t id = GICC0->IAR;
  if( id == GIC_SOURCE_TIMER0 ) {
    PL011_putc( UART0, 'T', true ); TIMER0->Timer1IntClr = 0x01;
    schedule(ctx, false);
  }
  GICC0->EOIR = id;

  return;
}


void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {

  switch( id ) {
    /*case 0x00 : { // 0x00 => yield()
      schedule( ctx );

      break;
    }*/

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }

    case 0x03 : { // 0x03 => fork

      memset( &pcb[ n_prog ], 0, sizeof( pcb_t ) );

      memcpy( &pcb[ n_prog].ctx, ctx, sizeof(ctx_t));

      //copy stack of parent to child
      int id = getCurrentIndex();
      memcpy( (void *) stacks[n_prog] - 0x00001000, (void *) stacks[id] - 0x00001000, 0x00001000);
      //child stack //parent stack, sizeof(stack));


      pcb[ n_prog ].pid      = n_prog + 1;
      pcb[ n_prog ].status   = STATUS_CREATED;
      pcb[ n_prog ].ctx.cpsr = 0x50; //just in case.

      //set stack pointer of child program same distance as parent sp.
      pcb[ n_prog ].ctx.sp = (uint32_t) stacks[n_prog] - (stacks[id] - ctx->sp);

      //pcb[ n_prog ].ctx.sp   = ( uint32_t ) stacks[n_prog]; //empty stack
      pcb[ n_prog ].priority = current->priority;
      pcb[ n_prog ].age = current->age;
      pcb[ n_prog ].ctx.gpr[0] = 0; //return 0 for child.
      // ??
      //pcb[ n_prog ].ctx.pc   = ctx->pc;
      ctx->gpr[0] = n_prog + 1; //return pid of child to parent.

      n_prog++;
      break;
    }

    case 0x04 : { // 0x04 => exit

      schedule(ctx, true);

      break;
    }

    case 0x05 : { // 0x05 => exec
      ctx->pc = (uint32_t) ctx->gpr[0];

      int id = getCurrentIndex();

      current->status = STATUS_READY;
      pcb[id].status = STATUS_EXECUTING;

      ctx->sp = stacks[id]; //reset stack to top.

      break;
    }

    case 0x06 : { // 0x06 => kill
      pid_t pid = ctx->gpr[0];

      for (int i=0; i<maxPrograms; i++){
        if (pid == pcb[i].pid){

          // char h = '0' + pid;
          // PL011_putc( UART0, h, true );
          if (pcb[i].status == STATUS_EXECUTING){
            // terminating itself
            schedule(ctx, true);
          } else {
            // terminating another process;
            pcb[i].status = STATUS_TERMINATED;
          }
          ctx->gpr[0] = 0; //return value of kill.
          return;

        }
      }

      ctx->gpr[0]=-1;
      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
