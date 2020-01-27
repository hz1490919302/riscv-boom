//******************************************************************************
// Copyright (c) 2015 - 2020, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Ring Microarchitecture Scheduler
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.Str

import FUConstants._
import boom.common._

class RingScheduler(numSlots: Int, columnDispatchWidth: Int)(implicit p: Parameters)
{
  val io = IO(new BoomBundle{
    val dis_uops = Flipped(Vec(coreWidth, DecoupledIO(new MicroOp)))
    val iss_uops = Output(Vec(coreWidth, Valid(new MicroOp)))

    val wakeups  = Input(Vec(coreWidth, Valid(UInt(pregSz.W))))
    val ld_miss  = Input(Bool()) // TODO use this

    val div_rdy  = Input(Bool()) // TODO do fu_types instead? Does it make a difference in synth?

    val brinfo   = Input(new BrResolutionInfo)
    val flush    = Input(Bool())
  })

  val numSlotsPerColumn = numSlots / coreWidth
  require (numSlots % coreWidth == 0)

  //----------------------------------------------------------------------------------------------------
  // Generate table of issue slots

  val issue_table = Seq.fill(coreWidth)( Seq.fill(numSlotsPerColumn)( Module(new RingIssueSlot) )
  val slots = VecInit(issue_table.map(col => VecInit(col.io)))

  //----------------------------------------------------------------------------------------------------
  // Selection

  val iss_sels = Wire(Vec(coreWidth, Vec(numSlotsPerColumn, Bool())))
  val sel_uops = Wire(Vec(coreWidth, new MicroOp))
  val sel_vals = Wire(Vec(coreWidth, Bool()))

  for (w <- 0 until coreWidth) {
    val col_reqs = slots(w).map(_.request)
    val col_uops = slots(w).map(_.uop)
    iss_sels(w) := PriorityEncoderOH(col_reqs)
    sel_uops(w) := Mux1H(col_uops, iss_sel(w))
    sel_vals(w) := iss_sels(w).reduce(_||_)
  }

  io.iss_uops := sel_uops

  //----------------------------------------------------------------------------------------------------
  // Arbitration

  val rrd_arb = Module(new RegisterReadArbiter)
  val exu_arb = Module(new ExecutionArbiter)
  val wb_arb  = Module(new WritebackArbiter)

  val arbiters = Seq(rrd_arb, exu_arb, wb_arb)

  val arb_gnts = Wire(Vec(coreWidth, Bool()))

  arbiters.foreach( arb ->
    arb.io.uops := sel_uops
    arb.io.reqs := sel_vals
    arb_gnts := arb.io.gnts
  ))

  //----------------------------------------------------------------------------------------------------
  // Grants

  for (w <- 0 until coreWidth) {
    for (i <- 0 until numSlotsPerColumn) {
      slots(w)(i).grant := iss_sels(w)(i) && arb_gnts(w)
    }
  }

  //----------------------------------------------------------------------------------------------------
  // Compaction

  for (w <- 0 until coreWidth) {
    val valids = slots(w).map(_.valid) ++ dis_valids(w)
    val uops = slots(w).map(_.uop) ++ dis_uops(w)
    val next_valids = slots(w).map(_.will_be_valid) ++ dis_valids(w)

    val max = columnDispatchWidth
    def Inc(count: UInt, inc: Bool) = Mux(inc && !count(max), count << 1, count)

    val counts = valids.scanLeft(1.U((max+1).W))((c,v) => Inc(c,!v))
    val sels = counts zip valids map { (c,v) => c.takeRight(max).map(b => b && v) }
                .takeRight(numSlotsPerColumn + coreWidth - 1)

    for (i <- 0 until numSlotsPerColumn) {
      val uop_sel = (0 until max).map(j => sels(i+j)(j))
      val will_be_valid = Mux1H(uop_sel, next_valids.slice(i+1,i+max+1))

      slot(w)(i).in_uop.bits  := Mux1H(uop_sel, uops.slice(i+1,i+max+1))
      slot(w)(i).in_uop.valid := will_be_valid
    }
  }
}
