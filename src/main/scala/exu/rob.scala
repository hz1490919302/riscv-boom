//******************************************************************************
// Copyright (c) 2013 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Re-order Buffer
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Bank the ROB, such that each "dispatch" group gets its own row of the ROB,
// and each instruction in the dispatch group goes to a different bank.
// We can compress out the PC by only saving the high-order bits!
//
// ASSUMPTIONS:
//    - dispatch groups are aligned to the PC.
//
// NOTES:
//    - Currently we do not compress out bubbles in the ROB.
//    - Exceptions are only taken when at the head of the commit bundle --
//      this helps deal with loads, stores, and refetch instructions.

package boom.exu

import scala.math.ceil
import chisel3._
import chisel3.util.{Valid, _}
import chisel3.experimental.chiselName
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.Str
import boom.common._
import boom.util._

/**
 * IO bundle to interact with the ROB
  *IO bundle与ROB交互
 *
 * @param numWakeupPorts number of wakeup ports to the rob    到rob的唤醒端口的数量
 * @param numFpuPorts number of fpu ports that will write back fflags    回写fflag的fpu端口数量
 */
class RobIo(
  val numWakeupPorts: Int,
  val numFpuPorts: Int
  )(implicit p: Parameters)  extends BoomBundle
{
  // Decode Stage
  // (Allocate, write instruction to ROB).     分配，写指令到ROB
  val enq_valids       = Input(Vec(coreWidth, Bool()))
  val enq_uops         = Input(Vec(coreWidth, new MicroOp()))
  val enq_partial_stall= Input(Bool()) // we're dispatching only a partial packet,
                                       // and stalling on the rest of it (don't
                                       // advance the tail ptr)  我们只调度了部分数据包，其余的暂停(不要推进尾部ptr)

  val xcpt_fetch_pc = Input(UInt(vaddrBitsExtended.W))

  val rob_tail_idx = Output(UInt(robAddrSz.W))     //rob尾部索引
  val rob_pnr_idx  = Output(UInt(robAddrSz.W))     //rob pnr索引
  val rob_head_idx = Output(UInt(robAddrSz.W))     //rob头部索引

  // Handle Branch Misspeculations                 //处理分支误预测
  val brupdate = Input(new BrUpdateInfo())

  // Write-back Stage
  // (Update of ROB)
  // Instruction is no longer busy and can be committed
  // 回写阶段
  // (ROB更新)
  // 指令不再繁忙，可以提交          //指令完成执行后，它们会通知ROB，并标记为不忙
  val wb_resps = Flipped(Vec(numWakeupPorts, Valid(new ExeUnitResp(xLen max fLen+1))))

  // Unbusying ports for stores.
  // +1 for fpstdata
  //为store解除端口的忙碌。
  // +1用于fpstdata
  val lsu_clr_bsy      = Input(Vec(memWidth + 1, Valid(UInt(robAddrSz.W))))

  // Port for unmarking loads/stores as speculation hazards..
  // 用于解除标记负载/存储的端口，有投机风险。
  val lsu_clr_unsafe   = Input(Vec(memWidth, Valid(UInt(robAddrSz.W))))


  // Track side-effects for debug purposes.
  // Also need to know when loads write back, whereas we don't need loads to unbusy.
  //跟踪用于调试的副作用。
  //还需要知道负载什么时候写回，而我们不需要负载来解除繁忙。
  val debug_wb_valids = Input(Vec(numWakeupPorts, Bool()))
  val debug_wb_wdata  = Input(Vec(numWakeupPorts, Bits(xLen.W)))

  val fflags = Flipped(Vec(numFpuPorts, new ValidIO(new FFlagsResp())))
  val lxcpt = Flipped(new ValidIO(new Exception())) // LSU

  // Commit stage (free resources; also used for rollback).   //提交阶段(空闲资源;也用于回滚)。
  val commit = Output(new CommitSignals())

  // tell the LSU that the head of the ROB is a load
  // (some loads can only execute once they are at the head of the ROB).
  // 告诉LSU，rob的头是一个load
  // (有些加载只有在处于ROB的头部时才能执行)。
  val com_load_is_at_rob_head = Output(Bool())

  // Communicate exceptions to the CSRFile            将异常传递给CSRFile
  val com_xcpt = Valid(new CommitExceptionSignals())

  // Let the CSRFile stall us (e.g., wfi).            让CSRFile暂停(例如，wfi)。
  val csr_stall = Input(Bool())

  // Flush signals (including exceptions, pipeline replays, and memory ordering failures)
  // to send to the frontend for redirection.
  //刷新信号(包括异常、管道重放和内存排序失败)
  //发送到前端进行重定向。
  val flush = Valid(new CommitExceptionSignals)

  // Stall Decode as appropriate           适当的暂停解码
  val empty = Output(Bool())
  val ready = Output(Bool()) // ROB is busy unrolling rename state...    ROB正忙着展开重命名状态…  //rename_state：该Entry的逻辑目标寄存器和物理目标寄存器分别是什么

  // Stall the frontend if we know we will redirect the PC     如果我们知道我们将重定向PC，停止前端
  val flush_frontend = Output(Bool())


  val debug_tsc = Input(UInt(xLen.W))

  val wakeup_yes = Input(Vec(numWakeupPorts+1,Bool()))
  val wakeup_already = Output(Vec(numWakeupPorts+1,Bool()))
  val wakeup_valid = Output(Vec(numWakeupPorts+1,Bool()))
  val wakeup_pdst = Output(Vec(numWakeupPorts+1,UInt(32.W)))
  val wakeup_row = Output(Vec(numWakeupPorts+1,UInt(32.W)))
  val wakeup_i = Output(Vec(4,UInt(32.W)))

  val risk_pdst = Input(Vec(memIssueParam.numEntries,UInt(32.W)))
  val risk_debug_inst = Input(Vec(memIssueParam.numEntries,UInt(32.W)))
  val risk_rob_idx = Input(Vec(memIssueParam.numEntries,UInt(32.W)))
  val idle_cycles = Input(UInt(32.W))
  val clear_risk_table = Output(Vec(10,UInt(log2Ceil(numIntPhysRegs).W)))  //10 is random
  val return_issue = Output(Bool())

  val risk_table = Input(Vec(numIntPhysRegs,Bool()))
  val fp_risk_table = Input(Vec(numFpPhysRegs,Bool()))
}

/**
 * Bundle to send commit signals across processor                   Bundle跨处理器发送提交信号
 */
class CommitSignals(implicit p: Parameters) extends BoomBundle
{
  //这些指令可能与架构上执行的指令不一致
  val valids      = Vec(retireWidth, Bool()) // These instructions may not correspond to an architecturally executed insn
  val arch_valids = Vec(retireWidth, Bool())
  val uops        = Vec(retireWidth, new MicroOp())
  val fflags      = Valid(UInt(5.W))

  // These come a cycle later   这些会在一个cycle后出现
  val debug_insts = Vec(retireWidth, UInt(32.W))

  // Perform rollback of rename state (in conjuction with commit.uops).   执行重命名状态回滚(与commit.uops一起)。
  val rbk_valids = Vec(retireWidth, Bool())
  val rollback   = Bool()

  val debug_wdata = Vec(retireWidth, UInt(xLen.W))
}

/**
 * Bundle to communicate exceptions to CSRFile            Bundle将异常传递给CSRFile
 *
 * TODO combine FlushSignals and ExceptionSignals (currently timed to different cycles).  TODO：结合FlushSignals和ExceptionSignals(当前计时到不同的周期)。
 */
class CommitExceptionSignals(implicit p: Parameters) extends BoomBundle
{
  val ftq_idx    = UInt(log2Ceil(ftqSz).W)    //ftq索引
  val edge_inst  = Bool()
  val is_rvc     = Bool()                     //是否是rvc
  val pc_lob     = UInt(log2Ceil(icBlockBytes).W)
  val cause      = UInt(xLen.W)               //异常原因
  val badvaddr   = UInt(xLen.W)               //bad虚拟地址
// The ROB needs to tell the FTQ if there's a pipeline flush (and what type)
// so the FTQ can drive the frontend with the correct redirected PC.
// ROB需要告诉FTQ是否有管道刷新(以及是哪种类型)
// 这样FTQ可以用正确的重定向PC驱动前端。
  val flush_typ  = FlushTypes()
}

/**
 * Tell the frontend the type of flush so it can set up the next PC properly.
  * 告诉前端flush的类型，以便它能正确设置下一个pc。
 */
object FlushTypes
{
  def SZ = 3
  def apply() = UInt(SZ.W)
  def none = 0.U
  def xcpt = 1.U // An exception occurred.                                 异常发生
  def eret = (2+1).U // Execute an environment return instruction.         执行一个环境返回指令
  def refetch = 2.U // Flush and refetch the head instruction.     刷新并重新获取head指令。
  def next = 4.U // Flush and fetch the next instruction.   刷新并获取下一条指令。

  def useCsrEvec(typ: UInt): Bool = typ(0) // typ === xcpt.U || typ === eret.U
  def useSamePC(typ: UInt): Bool = typ === refetch
  def usePCplus4(typ: UInt): Bool = typ === next

  def getType(valid: Bool, i_xcpt: Bool, i_eret: Bool, i_refetch: Bool): UInt = {
    val ret =
      Mux(!valid, none,
      Mux(i_eret, eret,
      Mux(i_xcpt, xcpt,
      Mux(i_refetch, refetch,
        next))))
    ret
  }
}

/**
 * Bundle of signals indicating that an exception occurred    指示发生异常的信号束
 */
class Exception(implicit p: Parameters) extends BoomBundle
{
  val uop = new MicroOp()
  val cause = Bits(log2Ceil(freechips.rocketchip.rocket.Causes.all.max+2).W)
  val badvaddr = UInt(coreMaxAddrBits.W)
}

/**
 * Bundle for debug ROB signals
 * These should not be synthesized!
  * *捆绑用于调试ROB信号
  * *这些不应该被合成!
 */
class DebugRobSignals(implicit p: Parameters) extends BoomBundle
{
  val state = UInt()
  val rob_head = UInt(robAddrSz.W)
  val rob_pnr = UInt(robAddrSz.W)
  val xcpt_val = Bool()
  val xcpt_uop = new MicroOp()
  val xcpt_badvaddr = UInt(xLen.W)
}

/**
  *  调度的uops（dis uops）写在ROB的底部（rob尾），而提交的uops（com uops）从rob头的顶部提交，并更新重命名状态。
  *  完成执行的Uop（wb uops）清除其繁忙位。注意：调度的uops一起写入同一ROB行，并连续位于内存中，从而使单个PC代表整个行。
  *
  * ROB必须知道每条飞行指令的PC。在以下情况下使用此信息：
  *
  * 任何指令都可能导致异常，其中必须知道“ exception pc”（epc）。
  * 分支和跳转指令需要知道自己的PC才能进行目标计算。
  * 跳转寄存器指令必须同时知道自己的PC和程序中下一条指令的PC，才能验证前端是否 预测了正确的JR目标。
  * 分支和跳转指令无需在流水线中传递PC，而是在寄存器读取阶段访问ROB的“PC文件”，以在分支单元中使用
  * 优化：1、每个ROB行仅存储一台PC。2、PC文件存储在两个bank中，从而允许单个读取端口同时读取两个连续的条目（与JR指令一起使用）。
  *
  * Reorder Buffer to keep track of dependencies and inflight instructions
 * 重排序缓冲区以跟踪依赖关系和飞行指令
  *
  * @param numWakeupPorts number of wakeup ports to the ROB   到ROB的唤醒端口的数量
 * @param numFpuPorts number of FPU units that will write back fflags   回写fflag的FPU单元的数量
 */
@chiselName
class Rob(
  val numWakeupPorts: Int,
  val numFpuPorts: Int
  )(implicit p: Parameters) extends BoomModule
{
  val io = IO(new RobIo(numWakeupPorts, numFpuPorts))

  // ROB Finite State Machine   rob有限状态机
  val s_reset :: s_normal :: s_rollback :: s_wait_till_empty :: Nil = Enum(4)
  val rob_state = RegInit(s_reset)

  //commit entries at the head, and unwind exceptions from the tail   在头部提交条目，从尾部展开异常
  val rob_head     = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_head_lsb = RegInit(0.U((1 max log2Ceil(coreWidth)).W)) // TODO: Accurately track head LSB (currently always 0) 准确跟踪头LSB(当前始终为0)
  val rob_head_idx = if (coreWidth == 1) rob_head else Cat(rob_head, rob_head_lsb)

  val rob_tail     = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_tail_lsb = RegInit(0.U((1 max log2Ceil(coreWidth)).W))
  val rob_tail_idx = if (coreWidth == 1) rob_tail else Cat(rob_tail, rob_tail_lsb)

  val rob_pnr      = RegInit(0.U(log2Ceil(numRobRows).W))
  val rob_pnr_lsb  = RegInit(0.U((1 max log2Ceil(coreWidth)).W))
  val rob_pnr_idx  = if (coreWidth == 1) rob_pnr  else Cat(rob_pnr , rob_pnr_lsb)

  val com_idx = Mux(rob_state === s_rollback, rob_tail, rob_head)          //回滚时，commit索引为tail；其他时候为head


  val maybe_full   = RegInit(false.B)
  val full         = Wire(Bool())
  val empty        = Wire(Bool())

  val will_commit         = Wire(Vec(coreWidth, Bool()))
  val can_commit          = Wire(Vec(coreWidth, Bool()))
  val can_throw_exception = Wire(Vec(coreWidth, Bool()))

  val rob_pnr_unsafe      = Wire(Vec(coreWidth, Bool())) // are the instructions at the pnr unsafe?  pnr的指令不安全吗?
  val rob_head_vals       = Wire(Vec(coreWidth, Bool())) // are the instructions at the head valid?  在head的指令有效吗
  val rob_tail_vals       = Wire(Vec(coreWidth, Bool())) // are the instructions at the tail valid? (to track partial row dispatches)尾部的指令有效吗?(跟踪部分行分派)
  val rob_head_uses_stq   = Wire(Vec(coreWidth, Bool())) //head是否使用stq
  val rob_head_uses_ldq   = Wire(Vec(coreWidth, Bool())) //head是否使用ldq
  val rob_head_fflags     = Wire(Vec(coreWidth, UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W)))

  val exception_thrown = Wire(Bool())         //异常抛出

  // exception info     异常信息
  // TODO compress xcpt cause size. Most bits in the middle are zero.   TODO压缩xcpt cause大小。中间的大多数位都是零。
  val r_xcpt_val       = RegInit(false.B)
  val r_xcpt_uop       = Reg(new MicroOp())
  val r_xcpt_badvaddr  = Reg(UInt(coreMaxAddrBits.W))
  io.flush_frontend := r_xcpt_val     //异常的值赋给io.flush_frontend

  //--------------------------------------------------
  // Utility  公用程序

  def GetRowIdx(rob_idx: UInt): UInt = {
    if (coreWidth == 1) return rob_idx
    else return rob_idx >> log2Ceil(coreWidth).U
  }
  def GetBankIdx(rob_idx: UInt): UInt = {
    if(coreWidth == 1) { return 0.U }
    else           { return rob_idx(log2Ceil(coreWidth)-1, 0).asUInt }
  }

  // **************************************************************************
  // Debug

  class DebugRobBundle extends BoomBundle
  {
    val valid      = Bool()
    val busy       = Bool()
    val unsafe     = Bool()
    val uop        = new MicroOp()
    val exception  = Bool()
  }
  val debug_entry = Wire(Vec(numRobEntries, new DebugRobBundle))  //ROB条目的数量(例如，对于R10k有32个条目)
  debug_entry := DontCare // override in statements below  在下面的语句中重写

  // **************************************************************************
  // --------------------------------------------------------------------------
  // **************************************************************************

  // Contains all information the PNR needs to find the oldest instruction which can't be safely speculated past.
  //包含了PNR查找不能安全推测的最古老指令所需的所有信息。
  val rob_unsafe_masked = WireInit(VecInit(Seq.fill(numRobRows << log2Ceil(coreWidth)){false.B}))

  // Used for trace port, for debug purposes only  用于跟踪端口，仅用于调试目的
  val rob_debug_inst_mem   = SyncReadMem(numRobRows, Vec(coreWidth, UInt(32.W)))
  val rob_debug_inst_wmask = WireInit(VecInit(0.U(coreWidth.W).asBools))
  val rob_debug_inst_wdata = Wire(Vec(coreWidth, UInt(32.W)))
  rob_debug_inst_mem.write(rob_tail, rob_debug_inst_wdata, rob_debug_inst_wmask)
  val rob_debug_inst_rdata = rob_debug_inst_mem.read(rob_head, will_commit.reduce(_||_))

  for (w <- 0 until coreWidth) {
    def MatchBank(bank_idx: UInt): Bool = (bank_idx === w.U)

    // one bank    //一个bank具有的标识
    val rob_val       = RegInit(VecInit(Seq.fill(numRobRows){false.B}))   //rob条目输入有效吗？
    val rob_bsy       = Reg(Vec(numRobRows, Bool()))                      //入口忙吗？
    val rob_unsafe    = Reg(Vec(numRobRows, Bool()))                      //rob_unsafe为这条指令是否“安全”，即肯定能被提交，与下文提到的PNR有关
    val rob_uop       = Reg(Vec(numRobRows, new MicroOp()))
    val rob_exception = Reg(Vec(numRobRows, Bool()))                      //是异常吗？
    val rob_predicated = Reg(Vec(numRobRows, Bool())) // Was this instruction predicated out?   //该指令是否已确定？
    val rob_fflags    = Mem(numRobRows, Bits(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W))

    val rob_debug_wdata = Mem(numRobRows, UInt(xLen.W))

    //-----------------------------------------------
    // Dispatch: Add Entry to ROB         调度：将条目添加到ROB

    rob_debug_inst_wmask(w) := io.enq_valids(w)
    rob_debug_inst_wdata(w) := io.enq_uops(w).debug_inst

    when (io.enq_valids(w)) {
      rob_val(rob_tail)       := true.B
      rob_bsy(rob_tail)       := !(io.enq_uops(w).is_fence ||
                                   io.enq_uops(w).is_fencei)
      rob_unsafe(rob_tail)    := io.enq_uops(w).unsafe
      rob_uop(rob_tail)       := io.enq_uops(w)
      rob_exception(rob_tail) := io.enq_uops(w).exception
      rob_predicated(rob_tail)   := false.B
      rob_fflags(rob_tail)    := 0.U

      //覆盖有效条目
      assert (rob_val(rob_tail) === false.B, "[rob] overwriting a valid entry.")
      assert ((io.enq_uops(w).rob_idx >> log2Ceil(coreWidth)) === rob_tail)
    } .elsewhen (io.enq_valids.reduce(_|_) && !rob_val(rob_tail)) {
      rob_uop(rob_tail).debug_inst := BUBBLE // just for debug purposes
    }


    //-----------------------------------------------
    // Writeback        写回
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    val wakeup_already = RegInit(VecInit(Seq.fill(numWakeupPorts+1){false.B}))
    val wakeup_valid = RegInit(VecInit(Seq.fill(numWakeupPorts+1){false.B}))
    val wakeup_pdst = RegInit(VecInit(Seq.fill(numWakeupPorts+1){0.U(32.W)}))
    val wakeup_row = RegInit(VecInit(Seq.fill(numWakeupPorts+1){0.U(32.W)}))

    for(i <- 0 until numWakeupPorts+1) {
      io.wakeup_already(i) := 0.B
      io.wakeup_valid(i) := 0.B
      io.wakeup_pdst(i) := wakeup_pdst(i)
      io.wakeup_row(i) := wakeup_row(i)
    }

    val clear_risk_table = RegInit(VecInit(Seq.fill(10){0.U(32.W)}))
    for(i <- 0 until 10) {
      io.clear_risk_table(i) := clear_risk_table(i)
    }
    var fflag = 0.U(32.W)

    var fflagx = 0.U(32.W)

    def already_before(j: Int) = {
        val xx = Wire(UInt(32.W))
        xx := 0.U
        for(i <- 0 until j) {
         // printf(p"wakeup_pdst(0)=${wakeup_pdst(0)} ")
         // printf(p"br_mask=${rob_uop(wakeup_row(0)).br_mask} ")
         // printf(p"wakeup_already(0)=${wakeup_already(0)} \n")
          //when(j.U === 1.U){
          when(wakeup_pdst(i) =/= 0.U && rob_uop(wakeup_row(i)).br_mask === 0.U && wakeup_already(i)){
             //xx := xx + 0.U
             /*printf(p"wakeup_pdst(0)=${wakeup_pdst(0)} ")
            printf(p"br_mask=${rob_uop(wakeup_row(0)).br_mask} ")
             printf(p" cycles=${io.idle_cycles} ")
             printf(p"wakeup_already(0)=${wakeup_already(0)} ")
             printf(p"xx=${xx} ")
             printf(p"i=${0} ")
             printf(p"j=${j} ")
             printf(p"abc \n")*/

          } .elsewhen(wakeup_pdst(i) =/= 0.U && rob_uop(wakeup_row(i)).br_mask === 0.U && !wakeup_already(i)){
             xx := 1.U
            /* printf(p"wakeup_pdst(0)=${wakeup_pdst(0)} ")
             printf(p"br_mask=${rob_uop(wakeup_row(0)).br_mask} ")
             printf(p" cycles=${io.idle_cycles} ")
             printf(p"wakeup_already(0)=${wakeup_already(0)} ")
             printf(p"xx=${xx} ")
             printf(p"i=${0} ")
             printf(p"j=${j} ")
             printf(p"123 \n")*/
          }
        }

        Mux(xx === 0.U,true.B,false.B)

    }


   // when(already_before(1) && wakeup_pdst(1) =/= 0.U &&  rob_uop(wakeup_row(1)).br_mask === 0.U  && !wakeup_already(1) && ((!rob_uop(wakeup_row(1)).fp_val && io.risk_table(rob_uop(wakeup_row(1)).pdst) === false.B) || (rob_uop(wakeup_row(1)).fp_val && io.fp_risk_table(rob_uop(wakeup_row(1)).pdst) === false.B))) { //等risk消除后,发出该唤醒
    //     printf(p" cycles=${io.idle_cycles} ")
    //     printf(" ABC \n")
   // }

      for(j <- 0 until numWakeupPorts+1) {
      when(already_before(j) && wakeup_pdst(j) =/= 0.U &&  rob_uop(wakeup_row(j)).br_mask === 0.U  && !wakeup_already(j) ) {  //等risk消除后,发出该唤醒 //&& ((!rob_uop(wakeup_row(j)).fp_val && io.risk_table(rob_uop(wakeup_row(j)).pdst) === false.B) || (rob_uop(wakeup_row(j)).fp_val && io.fp_risk_table(rob_uop(wakeup_row(j)).pdst) === false.B))
        io.wakeup_i(fflagx) := j.U
        io.wakeup_valid(j) := true.B
        io.wakeup_already(j) := true.B
        wakeup_valid(j) := true.B
        wakeup_already(j) := true.B

        printf(p" wakeup_row(j)=i=${wakeup_row(j)} ")
        printf(p" j=${j} ")
        printf(p" fflagx=${fflagx} ")
        printf(p" io.wakeup_i=${io.wakeup_i} ")
        printf(p" valid=${wakeup_valid}")
        printf(p" cycles=${io.idle_cycles} ")
        printf(p" pdst=${wakeup_pdst(j)} ")
        printf(p" wakeup_pdst.111111=${wakeup_pdst} \n")
        fflagx = (fflagx + 1.U)%4.U
      }
    }

    //br_mask==0时，消除risk
    for(i <- 0 until numRobRows) {
      //when(rob_uop(i).br_mask === 0.U  && rob_uop(i).risk === true.B && rob_uop(i).pdst =/= 0.U){  //
      when(rob_val(i) =/= 0.U && rob_uop(i).br_mask === 0.U  && ((!rob_uop(i).fp_val && io.risk_table(rob_uop(i).pdst) === true.B) || (rob_uop(i).fp_val && io.fp_risk_table(rob_uop(i).pdst) === true.B)) && rob_uop(i).pdst =/= 0.U){     // TODO
        //io.clear_risk_table(fflag) := Mux(rob_uop(i).fp_val,rob_uop(i).pdst + numIntPhysRegs.U,rob_uop(i).pdst) //通知risk_table消除该pdst的risk位
        clear_risk_table(fflag) := Mux(rob_uop(i).fp_val,rob_uop(i).pdst + numIntPhysRegs.U,rob_uop(i).pdst) //通知risk_table消除该pdst的risk位

        //rob_uop(i).risk := false.B
        printf(p" start mask=${rob_uop(i).br_mask} ")
        printf(" inst=0x%x ", rob_uop(i).debug_inst)
        printf(" pc=0x%x ", rob_uop(i).debug_pc)
        printf(p" i=${i} ")
        printf(p" rob_val=${rob_val(i)} ")
        printf(p" rob_head=${rob_head} ")
        printf(p" rob_tail=${rob_tail} ")
        printf(p" rob_head_idx=${rob_head_idx} ")
        printf(p" rob_tail_idx=${rob_tail_idx} ")
        printf(p" cycles=${io.idle_cycles} ")
        printf(p" pdst_clear=${rob_uop(i).pdst} ")
        printf(p" fflag=${fflag} ")
        printf(p" clear risk table\n ")

        /*when(rob_uop(i).debug_inst(15,0) === 0x97baL.U) {
          printf(p" i=${i}" )
          printf(p" 0x97baL: fflag = ${fflag} ")
          printf(p" cycles=${io.idle_cycles} ")
          printf(p" risk_table=${io.risk_table} \n")
        }*/

        fflag = (fflag + 1.U)%10.U

        //当rob中的记录的推测性指令不在是推测的时，发出唤醒信号       //目前只是load唤醒，TODO

      }
    }

    for(i <- 0 until 10) {
      when(clear_risk_table(i) =/= 0.U){
        clear_risk_table(i) := 0.U
      }
    }


    //当推测性load解除推测性，并且已经唤醒/广播后,将该load在rob中的唤醒项清除
    val wakeup_yes1 = Wire(Vec(numWakeupPorts+1,Bool()))
    for(i <- 0 until numWakeupPorts+1) {
      wakeup_yes1(i) := io.wakeup_yes(i)
      when(wakeup_already(i) && wakeup_valid(i) && io.wakeup_yes(i)) {
        wakeup_already((i)%(numWakeupPorts+1)) := false.B
        wakeup_valid((i)%(numWakeupPorts+1)) := false.B
        io.wakeup_already((i)%(numWakeupPorts+1)) := false.B
        io.wakeup_valid((i)%(numWakeupPorts+1)) := false.B
        wakeup_pdst((i)%(numWakeupPorts+1)) := 0.U
        wakeup_row((i)%(numWakeupPorts+1)) := 0.U
        //fflagx = (fflagx - 1.U + 4.U)%4.U
      }
    }

    //告诉issue-slot，已收到其发来的唤醒指令信息
    val return_issue  = Reg(Bool())     //不该是rob.numWakeupPorts+1
    return_issue := false.B
    io.return_issue := return_issue
    when(return_issue === true.B){
      return_issue := RegNext(false.B)
    }

    var flag1 = 0.U(32.W)
    //rob等待risk信号,来自issue-slot
    for(i <- 0 until memIssueParam.numEntries) {
      when(io.risk_pdst(i) =/= 0.U) {
        printf(" found-risk-debug-inst ")
        printf(p" risk_pdst=${io.risk_pdst(i)} ")
        printf(p" cycles=${io.idle_cycles} ")
        printf(p" risk_rob_idx=${io.risk_rob_idx(i)} \n")
        
         when(!(( io.risk_pdst(i) === io.risk_pdst((i-1+memIssueParam.numEntries)%memIssueParam.numEntries)) && (io.risk_debug_inst(i) === io.risk_debug_inst((i-1+memIssueParam.numEntries)%memIssueParam.numEntries)) && (io.risk_rob_idx(i) === io.risk_rob_idx((i-1+memIssueParam.numEntries)%memIssueParam.numEntries)))){
        
        
         //不应该是numWakeupPorts+1,暂时图方便
          when(wakeup_pdst(flag1) === 0.U && wakeup_row(flag1) === 0.U && !return_issue) {
            wakeup_pdst(flag1) := io.risk_pdst(i)
            wakeup_row(flag1) := io.risk_rob_idx(i)
            return_issue := true.B
            printf(p" i=${i} ")
            printf(p" flag1-before=${flag1} ")
            flag1 = (flag1 + 1.U)%(numWakeupPorts.U + 1.U)
            printf(p" flag1-after=${flag1} \n")
          }
        
        }
      }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (i <- 0 until numWakeupPorts) {
      val wb_resp = io.wb_resps(i)
      val wb_uop = wb_resp.bits.uop  //写回微操作
      val row_idx = GetRowIdx(wb_uop.rob_idx)

      //当第一个推测性load到来，记录该load的唤醒pdst，以及该load在rob中的行

      when (wb_resp.valid && MatchBank(GetBankIdx(wb_uop.rob_idx))) {  //wb返回值有效并且bank匹配
        rob_bsy(row_idx)      := false.B            //rob标记为不忙
        rob_unsafe(row_idx)   := false.B            //rob标记为安全
        rob_predicated(row_idx)  := wb_resp.bits.predicated       //是否断定？ rob标记为断定
      }
      // TODO check that fflags aren't overwritten
      // TODO check that the wb is to a valid ROB entry, give it a time stamp
//        assert (!(wb_resp.valid && MatchBank(GetBankIdx(wb_uop.rob_idx)) &&
//                  wb_uop.fp_val && !(wb_uop.is_load || wb_uop.is_store) &&
//                  rob_exc_cause(row_idx) =/= 0.U),
//                  "FP instruction writing back exc bits is overriding an existing exception.")
    }

    // Stores have a separate method to clear busy bits       store有一个单独的方法来清除繁忙的位、清除不安全的位
    for (clr_rob_idx <- io.lsu_clr_bsy) {
      when (clr_rob_idx.valid && MatchBank(GetBankIdx(clr_rob_idx.bits))) {
        val cidx = GetRowIdx(clr_rob_idx.bits)
        rob_bsy(cidx)    := false.B
        rob_unsafe(cidx) := false.B
        assert (rob_val(cidx) === true.B, "[rob] store writing back to invalid entry.")
        assert (rob_bsy(cidx) === true.B, "[rob] store writing back to a not-busy entry.")
      }
    }
    for (clr <- io.lsu_clr_unsafe) {
      when (clr.valid && MatchBank(GetBankIdx(clr.bits))) {
        val cidx = GetRowIdx(clr.bits)
        rob_unsafe(cidx) := false.B
      }
    }


    //-----------------------------------------------
    // Accruing fflags      积累fflags
    for (i <- 0 until numFpuPorts) {
      val fflag_uop = io.fflags(i).bits.uop
      when (io.fflags(i).valid && MatchBank(GetBankIdx(fflag_uop.rob_idx))) {
        rob_fflags(GetRowIdx(fflag_uop.rob_idx)) := io.fflags(i).bits.flags
      }
    }

    //-----------------------------------------------------
    // Exceptions  异常(cause位被压缩并存储在其他地方)
    // (the cause bits are compressed and stored elsewhere)

    //当提交head的指令异常时，将处理异常。然后flush管道，清空ROB。
    //重命名映射表必须复位到 代表真实的，非投机性 的提交状态。然后将前端定向到适当的PC。
    //如果是体系结构异常，则将异常指令的PC（称为异常向量）发送到控制/状态寄存器（CSR）文件。
    //如果是微体系结构异常（例如，加载/存储顺序错误推测），则会重新获取失败的指令，并且可以重新开始执行。

    //RV64G ISA提供了相对较少的异常源：
    //1、加载/存储单元  页面错误
    //2、分支单元   未对齐的提取
    //3、解码阶段    在将指令分派到ROB之前，可以处理所有其他异常和中断
    //请注意，内存排序推测错误也源自加载/存储单元，在BOOM管道中被视为异常（实际上，它们仅会导致管道“重试”）。

    when (io.lxcpt.valid && MatchBank(GetBankIdx(io.lxcpt.bits.uop.rob_idx))) {
      rob_exception(GetRowIdx(io.lxcpt.bits.uop.rob_idx)) := true.B
      when (io.lxcpt.bits.cause =/= MINI_EXCEPTION_MEM_ORDERING) {    //MINI_EXCEPTION_MEM_ORDERING:内存歧义错误猜测发生
        // In the case of a mem-ordering failure, the failing load will have been marked safe already.
        //在内存排序失败的情况下，失败的负载已经被标记为安全。
        assert(rob_unsafe(GetRowIdx(io.lxcpt.bits.uop.rob_idx)),
          "An instruction marked as safe is causing an exception")
      }
    }
    can_throw_exception(w) := rob_val(rob_head) && rob_exception(rob_head)   //rob head是异常时才抛出

    //-----------------------------------------------
    // Commit or Rollback   提交或回滚

    // Can this instruction commit? (the check for exceptions/rob_state happens later).
    // 此指令能提交吗?(稍后检查 异常/rob_state)。
    can_commit(w) := rob_val(rob_head) && !(rob_bsy(rob_head)) && !io.csr_stall  //可以提交： rob head有效、rob head不在busy、没有csr.stall


    // use the same "com_uop" for both rollback AND commit         对回滚和提交使用相同的“com_uop”
    // 执行提交
    // Perform Commit
    io.commit.valids(w) := will_commit(w)
    io.commit.arch_valids(w) := will_commit(w) && !rob_predicated(com_idx)      //架构有效性：将提交并且尚未断定
    io.commit.uops(w)   := rob_uop(com_idx)
    io.commit.debug_insts(w) := rob_debug_inst_rdata(w)

    // We unbusy branches in b1, but its easier to mark the taken/provider src in b2,
    // when the branch might be committing
    //我们在b1中取消了分支busy，但是当分支可能正在提交时，在b2中标记接受/提供者src更容易
    when (io.brupdate.b2.mispredict &&
      MatchBank(GetBankIdx(io.brupdate.b2.uop.rob_idx)) &&
      GetRowIdx(io.brupdate.b2.uop.rob_idx) === com_idx) {    //分支误预测 && bank匹配 && 分支正在提交
      io.commit.uops(w).debug_fsrc := BSRC_C
      io.commit.uops(w).taken      := io.brupdate.b2.taken         //分支taken赋给提交微操作的taken
    }


    // Don't attempt to rollback the tail's row when the rob is full.  当rob已满时，不要试图回滚rob tail的row。
    val rbk_row = rob_state === s_rollback && !full

    io.commit.rbk_valids(w) := rbk_row && rob_val(com_idx) && !(enableCommitMapTable.B)
    io.commit.rollback := (rob_state === s_rollback)

    assert (!(io.commit.valids.reduce(_||_) && io.commit.rbk_valids.reduce(_||_)),
      "com_valids and rbk_valids are mutually exclusive")
    //com_valids和rbk_valids是互斥的

    when (rbk_row) {  //回滚时，commit为false
      rob_val(com_idx)       := false.B
      rob_exception(com_idx) := false.B
    }

    if (enableCommitMapTable) {
      when (RegNext(exception_thrown)) {
        for (i <- 0 until numRobRows) {
          rob_val(i) := false.B
          rob_bsy(i) := false.B
          rob_uop(i).debug_inst := BUBBLE
        }
      }
    }

    // -----------------------------------------------
    // Kill speculated entries on branch mispredict  杀死分支预测错误的推测条目
    for (i <- 0 until numRobRows) {
      val br_mask = rob_uop(i).br_mask           //该Entry的指令属于哪个处于预测中的分支上

      //kill instruction if mispredict & br mask match        如果预测错误和br掩码匹配，则终止指令
      when (IsKilledByBranch(io.brupdate, br_mask))
      {
        rob_val(i) := false.B
        rob_uop(i.U).debug_inst := BUBBLE
      } .elsewhen (rob_val(i)) {
        // clear speculation bit even on correct speculation   即使正确的猜测也清除推测位
        rob_uop(i).br_mask := GetNewBrMask(io.brupdate, br_mask)
      }
    }


    // Debug signal to figure out which prediction structure
    // or core resolved a branch correctly   //调试信号以找出哪个预测结构或核心正确解析了一个分支
    when (io.brupdate.b2.mispredict &&
      MatchBank(GetBankIdx(io.brupdate.b2.uop.rob_idx))) {
      rob_uop(GetRowIdx(io.brupdate.b2.uop.rob_idx)).debug_fsrc := BSRC_C
      rob_uop(GetRowIdx(io.brupdate.b2.uop.rob_idx)).taken      := io.brupdate.b2.taken
    }

    // -----------------------------------------------
    // Commit           提交
    when (will_commit(w)) {
      rob_val(rob_head) := false.B
    }

    // -----------------------------------------------
    // Outputs          输出
    rob_head_vals(w)     := rob_val(rob_head)
    rob_tail_vals(w)     := rob_val(rob_tail)
    rob_head_fflags(w)   := rob_fflags(rob_head)
    rob_head_uses_stq(w) := rob_uop(rob_head).uses_stq
    rob_head_uses_ldq(w) := rob_uop(rob_head).uses_ldq

    //------------------------------------------------
    // Invalid entries are safe; thrown exceptions are unsafe.  无效的输入是安全的；抛出异常是不安全的。
    for (i <- 0 until numRobRows) {
      rob_unsafe_masked((i << log2Ceil(coreWidth)) + w) := rob_val(i) && (rob_unsafe(i) || rob_exception(i))
    }
    // Read unsafe status of PNR row.   读取PNR行的不安全状态。
    rob_pnr_unsafe(w) := rob_val(rob_pnr) && (rob_unsafe(rob_pnr) || rob_exception(rob_pnr))

    // -----------------------------------------------
    // debugging write ports that should not be synthesized   调试写端口that不应综合的
    when (will_commit(w)) {
      rob_uop(rob_head).debug_inst := BUBBLE
    } .elsewhen (rbk_row)
    {
      rob_uop(rob_tail).debug_inst := BUBBLE
    }

    //--------------------------------------------------
    // Debug: for debug purposes, track side-effects to all register destinations
    // 调试：出于调试目的，跟踪所有寄存器目标的副作用

    for (i <- 0 until numWakeupPorts) {
      val rob_idx = io.wb_resps(i).bits.uop.rob_idx
      when (io.debug_wb_valids(i) && MatchBank(GetBankIdx(rob_idx))) {
        rob_debug_wdata(GetRowIdx(rob_idx)) := io.debug_wb_wdata(i)
      }
      val temp_uop = rob_uop(GetRowIdx(rob_idx))

      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               !rob_val(GetRowIdx(rob_idx))),
               "[rob] writeback (" + i + ") occurred to an invalid ROB entry.")
      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               !rob_bsy(GetRowIdx(rob_idx))),
               "[rob] writeback (" + i + ") occurred to a not-busy ROB entry.")
      assert (!(io.wb_resps(i).valid && MatchBank(GetBankIdx(rob_idx)) &&
               temp_uop.ldst_val && temp_uop.pdst =/= io.wb_resps(i).bits.uop.pdst),
               "[rob] writeback (" + i + ") occurred to the wrong pdst.")
    }
    io.commit.debug_wdata(w) := rob_debug_wdata(rob_head)

  } //for (w <- 0 until coreWidth)

  // **************************************************************************
  // --------------------------------------------------------------------------
  // **************************************************************************

  // -----------------------------------------------
  // Commit Logic
  // need to take a "can_commit" array, and let the first can_commits commit
  // previous instructions may block the commit of younger instructions in the commit bundle
  // e.g., exception, or (valid && busy).
  // Finally, don't throw an exception if there are instructions in front of
  // it that want to commit (only throw exception when head of the bundle).
  //提交的逻辑
  //需要取一个“can_commit”数组，并让第一个can_commit提交
  //之前的指令可能会block提交包中更年轻指令的提交,例如，例外，或(有效&&繁忙)。
  //最后，如果前面有想要提交的指令，不要抛出异常(仅在bundle头时抛出异常)。

  // 当在指令提交head不再忙（并且无异常），可以提交，即其更改机器的结构状态都可见
  // 对于超标量提交，将分析整个ROB行中是否没有繁忙的指令（因此，最多可在单个周期中提交整个ROB行）。
  // ROB会贪婪地提交每行尽可能多的指令以尽快释放资源。但是，ROB（当前）不支持跨多行查找可提交的指令。

  //仅在提交store后，才可以将其发送到内存。
  //当一条指令（写入寄存器）提交时，它会释放过时的物理目标寄存器。然后，可以将陈旧的pdst随意重新分配给新指令。

  var block_commit = (rob_state =/= s_normal) && (rob_state =/= s_wait_till_empty) || RegNext(exception_thrown) || RegNext(RegNext(exception_thrown))
  var will_throw_exception = false.B
  var block_xcpt   = false.B

  for (w <- 0 until coreWidth) {
    will_throw_exception = (can_throw_exception(w) && !block_commit && !block_xcpt) || will_throw_exception               //将抛出异常

    will_commit(w)       := can_commit(w) && !can_throw_exception(w) && !block_commit                      //将提交
    block_commit         = (rob_head_vals(w) &&
                           (!can_commit(w) || can_throw_exception(w))) || block_commit                     //阻塞提交
    block_xcpt           = will_commit(w)                                                                  //阻塞异常
  }

  // Note: exception must be in the commit bundle.
  // Note: exception must be the first valid instruction in the commit bundle.
  //注意:异常必须在提交bundle中。
  //注意:exception必须是commit bundle中的第一个有效指令。
  exception_thrown := will_throw_exception
  val is_mini_exception = io.com_xcpt.bits.cause === MINI_EXCEPTION_MEM_ORDERING
  io.com_xcpt.valid := exception_thrown && !is_mini_exception         //异常抛出并且不是mini异常，则将异常传递给CSRFile有效
  io.com_xcpt.bits.cause := r_xcpt_uop.exc_cause

  io.com_xcpt.bits.badvaddr := Sext(r_xcpt_badvaddr, xLen)
  val insn_sys_pc2epc =
    rob_head_vals.reduce(_|_) && PriorityMux(rob_head_vals, io.commit.uops.map{u => u.is_sys_pc2epc})  //是ECall或断点?——都设置EPC到PC。

  val refetch_inst = exception_thrown || insn_sys_pc2epc           //重新获取指令
  val com_xcpt_uop = PriorityMux(rob_head_vals, io.commit.uops)    //PriorityMux什么意思？
  io.com_xcpt.bits.ftq_idx   := com_xcpt_uop.ftq_idx
  io.com_xcpt.bits.edge_inst := com_xcpt_uop.edge_inst
  io.com_xcpt.bits.is_rvc    := com_xcpt_uop.is_rvc
  io.com_xcpt.bits.pc_lob    := com_xcpt_uop.pc_lob

  //有些指令需要清理它们后面的管道
  val flush_commit_mask = Range(0,coreWidth).map{i => io.commit.valids(i) && io.commit.uops(i).flush_on_commit}
  val flush_commit = flush_commit_mask.reduce(_|_)
  val flush_val = exception_thrown || flush_commit

  assert(!(PopCount(flush_commit_mask) > 1.U),
    "[rob] Can't commit multiple flush_on_commit instructions on one cycle")

  val flush_uop = Mux(exception_thrown, com_xcpt_uop, Mux1H(flush_commit_mask, io.commit.uops))

  // delay a cycle for critical path considerations      出于关键路径考虑而延迟一个周期
  io.flush.valid          := flush_val
  io.flush.bits.ftq_idx   := flush_uop.ftq_idx
  io.flush.bits.pc_lob    := flush_uop.pc_lob
  io.flush.bits.edge_inst := flush_uop.edge_inst
  io.flush.bits.is_rvc    := flush_uop.is_rvc
  io.flush.bits.flush_typ := FlushTypes.getType(flush_val,
                                                exception_thrown && !is_mini_exception,
                                                flush_commit && flush_uop.uopc === uopERET,
                                                refetch_inst)


  // -----------------------------------------------
  // FP Exceptions
  // send fflags bits to the CSRFile to accrue
  // FP异常
  // 发送fflag位到CSRFile以累积

  val fflags_val = Wire(Vec(coreWidth, Bool()))
  val fflags     = Wire(Vec(coreWidth, UInt(freechips.rocketchip.tile.FPConstants.FLAGS_SZ.W)))

  for (w <- 0 until coreWidth) {
    fflags_val(w) :=
      io.commit.valids(w) &&
      io.commit.uops(w).fp_val &&
      !io.commit.uops(w).uses_stq    //fflag= commit有效 && commit是一个浮点指令 && 不使用stq

    fflags(w) := Mux(fflags_val(w), rob_head_fflags(w), 0.U)

    //已提交的非fp指令具有非零的fflag位
    assert (!(io.commit.valids(w) &&
             !io.commit.uops(w).fp_val &&
             rob_head_fflags(w) =/= 0.U),
             "Committed non-FP instruction has non-zero fflag bits.")
    //提交的FP加载或存储具有非零的fflag位
    assert (!(io.commit.valids(w) &&
             io.commit.uops(w).fp_val &&
             (io.commit.uops(w).uses_ldq || io.commit.uops(w).uses_stq) &&
             rob_head_fflags(w) =/= 0.U),
             "Committed FP load or store has non-zero fflag bits.")
  }
  io.commit.fflags.valid := fflags_val.reduce(_|_)
  io.commit.fflags.bits  := fflags.reduce(_|_)

  // -----------------------------------------------
  // Exception Tracking Logic
  // only store the oldest exception, since only one can happen!
  //异常跟踪逻辑
  //只存储最旧的异常，因为只有一个异常会发生!如果此指令到达ROB的开头，则会引发异常。
  //每个ROB条目都用一位标记，以指示该指令是否遇到异常行为，但是仅针对已知最早的已知指令跟踪附加异常状态（例如，错误的虚拟地址和异常原因）

  val next_xcpt_uop = Wire(new MicroOp())
  next_xcpt_uop := r_xcpt_uop
  val enq_xcpts = Wire(Vec(coreWidth, Bool()))   //异常入队列
  for (i <- 0 until coreWidth) {
    enq_xcpts(i) := io.enq_valids(i) && io.enq_uops(i).exception
  }

  //当没有flush\无异常\不回滚时
  when (!(io.flush.valid || exception_thrown) && rob_state =/= s_rollback) {
    when (io.lxcpt.valid) {
      val new_xcpt_uop = io.lxcpt.bits.uop

      when (!r_xcpt_val || IsOlder(new_xcpt_uop.rob_idx, r_xcpt_uop.rob_idx, rob_head_idx)) {   //new_xcpt_uop是否更老
        r_xcpt_val              := true.B
        next_xcpt_uop           := new_xcpt_uop
        next_xcpt_uop.exc_cause := io.lxcpt.bits.cause
        r_xcpt_badvaddr         := io.lxcpt.bits.badvaddr
      }
    } .elsewhen (!r_xcpt_val && enq_xcpts.reduce(_|_)) {
      val idx = enq_xcpts.indexWhere{i: Bool => i}          //输出 输出为真的第一个元素的索引。

      // if no exception yet, dispatch exception wins       如果还没有异常，调度异常将获胜
      r_xcpt_val      := true.B
      next_xcpt_uop   := io.enq_uops(idx)
      r_xcpt_badvaddr := AlignPCToBoundary(io.xcpt_fetch_pc, icBlockBytes) | io.enq_uops(idx).pc_lob

    }
  }

  //更新最老异常
  r_xcpt_uop         := next_xcpt_uop
  r_xcpt_uop.br_mask := GetNewBrMask(io.brupdate, next_xcpt_uop)
  when (io.flush.valid || IsKilledByBranch(io.brupdate, next_xcpt_uop)) {
    r_xcpt_val := false.B
  }

  //ROB试图抛出一个异常，但是它没有一个有效的xcpt_cause
  assert (!(exception_thrown && !r_xcpt_val),
    "ROB trying to throw an exception, but it doesn't have a valid xcpt_cause")

  //rob是空的，但他相信有一个突出的异常。
  assert (!(empty && r_xcpt_val),
    "ROB is empty, but believes it has an outstanding exception.")

  //ROB抛出了一个异常，但是存储的异常信息的rob_idx与rob_head不匹配”
  assert (!(will_throw_exception && (GetRowIdx(r_xcpt_uop.rob_idx) =/= rob_head)),
    "ROB is throwing an exception, but the stored exception information's " +
    "rob_idx does not match the rob_head")

  // -----------------------------------------------
  // ROB Head Logic

  // remember if we're still waiting on the rest of the dispatch packet, and prevent
  // the rob_head from advancing if it commits a partial parket before we
  // dispatch the rest of it.
  // update when committed ALL valid instructions in commit_bundle
  //rob head逻辑
  //记住，如果我们仍然在等待分派包的其余部分，并且如果rob_head在我们分派其余部分之前提交了部分parket，那么它将阻止rob_head前进。
  //在提交commit_bundle中的所有有效指令时进行更新

  val rob_deq = WireInit(false.B)
  val r_partial_row = RegInit(false.B)

  when (io.enq_valids.reduce(_|_)) {
    r_partial_row := io.enq_partial_stall
  }

  // 完成commit: commit有效 && will_commit ^ rob_head_vals=0 && !（head=tail && 未满 && 部分commit）    //逻辑是？
  val finished_committing_row =
    (io.commit.valids.asUInt =/= 0.U) &&
    ((will_commit.asUInt ^ rob_head_vals.asUInt) === 0.U) &&
    !(r_partial_row && rob_head === rob_tail && !maybe_full)

  when (finished_committing_row) {
    rob_head     := WrapInc(rob_head, numRobRows)    //递增输入值，必要时将其封装。
    rob_head_lsb := 0.U
    rob_deq      := true.B
  } .otherwise {
    rob_head_lsb := OHToUInt(PriorityEncoderOH(rob_head_vals.asUInt))
  }

  // -----------------------------------------------
  // ROB Point-of-No-Return (PNR) Logic
  // Acts as a second head, but only waits on busy instructions which might cause misspeculation.
  // TODO is it worth it to add an extra 'parity' bit to all rob pointer logic?
  // Makes 'older than' comparisons ~3x cheaper, in case we're going to use the PNR to do a large number of those.
  // Also doesn't require the rob tail (or head) to be exported to whatever we want to compare with the PNR.
  // ROB不返回点(PNR)逻辑
  //充当第二个头，但只等待可能引起错误猜测的繁忙指令。
  //TODO : 值得为所有rob指针逻辑添加一个额外的奇偶校验位吗?
  //使得'older than'的比较便宜3倍，以防我们使用PNR进行大量的比较。
  //也不需要将rob tail(或head)导出到我们想要与PNR进行比较的地方。

  // pnr head在ROB提交head之前运行,标记下一条可能被错误推测或产生异常的指令
  // 这些包括未解决的分支和未翻译的内存操作.因此,该指令that在 commit head的前方与PNR head的后面 都保证非投机性的,即使他们还没有写回

  if (enableFastPNR) {    //启用快速PNR
    val unsafe_entry_in_rob = rob_unsafe_masked.reduce(_||_)   //在rob中不安全的条目
    val next_rob_pnr_idx = Mux(unsafe_entry_in_rob,
                               AgePriorityEncoder(rob_unsafe_masked, rob_head_idx),
                               rob_tail << log2Ceil(coreWidth) | PriorityEncoder(~rob_tail_vals.asUInt))
    //AgePriorityEncoder:返回head后的最低位位置。     PriorityEncoder:返回输入位向量的最低有效高位的位位置。
    rob_pnr := next_rob_pnr_idx >> log2Ceil(coreWidth)
    if (coreWidth > 1)
      rob_pnr_lsb := next_rob_pnr_idx(log2Ceil(coreWidth)-1, 0)
  } else {
    // Distinguish between PNR being at head/tail when ROB is full.
    // Works the same as maybe_full tracking for the ROB tail.
    //当ROB满的时候，区分PNR是在头还是尾。
    //工作方式与ROB tail的maybe_full跟踪相同。
    val pnr_maybe_at_tail = RegInit(false.B)

    val safe_to_inc = rob_state === s_normal || rob_state === s_wait_till_empty
    //即当PNR当前所指向的行的所有指令的unsafe都为false时（同时PNR指向的不是tail或者ROB已满等情况），PNR会向下指向一行。
    val do_inc_row  = !rob_pnr_unsafe.reduce(_||_) && (rob_pnr =/= rob_tail || (full && !pnr_maybe_at_tail)) //（pnr不等于tail 或 rob满了但是pnr不可能在tail） && pnr安全
    when (empty && io.enq_valids.asUInt =/= 0.U) {//rob为空 且 队列有效
      // Unforunately for us, the ROB does not use its entries in monotonically
      //  increasing order, even in the case of no exceptions. The edge case
      //  arises when partial rows are enqueued and committed, leaving an empty
      //  ROB.
      //对我们来说，不幸的是，即使在没有异常的情况下，ROB也不会以单调递增的顺序使用它的条目。当部分行进入队列并提交，ROB为空时，就会出现边缘情况。
      rob_pnr     := rob_head
      rob_pnr_lsb := PriorityEncoder(io.enq_valids)
    } .elsewhen (safe_to_inc && do_inc_row) {  //rob状态可以安全递增pnr && 可以做pnr增加
      rob_pnr     := WrapInc(rob_pnr, numRobRows)
      rob_pnr_lsb := 0.U
    } .elsewhen (safe_to_inc && (rob_pnr =/= rob_tail || (full && !pnr_maybe_at_tail))) { //rob状态可以安全递增pnr && pnr不安全
      rob_pnr_lsb := PriorityEncoder(rob_pnr_unsafe)
    } .elsewhen (safe_to_inc && !full && !empty) {  //rob状态可以安全递增pnr && 非空 && 非满
      rob_pnr_lsb := PriorityEncoder(rob_pnr_unsafe.asUInt | ~MaskLower(rob_tail_vals.asUInt))
    } .elsewhen (full && pnr_maybe_at_tail) {       //rob满并且pnr可能在尾部
      rob_pnr_lsb := 0.U
    }

    pnr_maybe_at_tail := !rob_deq && (do_inc_row || pnr_maybe_at_tail)     //pnr_maybe_at_tail = rob未出队 && （可以pnr增加 或 pnr可能在尾部）
  }

  // Head overrunning PNR likely means an entry hasn't been marked as safe when it should have been.
  //头部超过PNR可能意味着入口没有被标记为安全，而它应该是安全的。
  assert(!IsOlder(rob_pnr_idx, rob_head_idx, rob_tail_idx) || rob_pnr_idx === rob_tail_idx)

  // PNR overrunning tail likely means an entry has been marked as safe when it shouldn't have been.
  //PNR过尾很可能意味着条目在本不应该被标记为安全的情况下被标记为安全。
  assert(!IsOlder(rob_tail_idx, rob_pnr_idx, rob_head_idx) || full)

  // -----------------------------------------------
  // ROB Tail Logic   ROB tail逻辑

  val rob_enq = WireInit(false.B)

  when (rob_state === s_rollback && (rob_tail =/= rob_head || maybe_full)) {  //rob状态=回滚 && （tail!=head或rob可能满了）
    // Rollback a row  回滚一行
    rob_tail     := WrapDec(rob_tail, numRobRows)
    rob_tail_lsb := (coreWidth-1).U
    rob_deq := true.B
  } .elsewhen (rob_state === s_rollback && (rob_tail === rob_head) && !maybe_full) {  //rob状态=回滚 && tail=head && rob不可能满
    // Rollback an entry   回滚一个条目
    rob_tail_lsb := rob_head_lsb
  } .elsewhen (io.brupdate.b2.mispredict) {      //误预测
    rob_tail     := WrapInc(GetRowIdx(io.brupdate.b2.uop.rob_idx), numRobRows)
    rob_tail_lsb := 0.U
  } .elsewhen (io.enq_valids.asUInt =/= 0.U && !io.enq_partial_stall) {    //入队列有效 && 没有“只调度了部分数据包，其余的暂停”
    rob_tail     := WrapInc(rob_tail, numRobRows)
    rob_tail_lsb := 0.U
    rob_enq      := true.B
  } .elsewhen (io.enq_valids.asUInt =/= 0.U && io.enq_partial_stall) {     //入队列有效 && 只调度了部分数据包，其余的暂停
    rob_tail_lsb := PriorityEncoder(~MaskLower(io.enq_valids.asUInt))
  }


  if (enableCommitMapTable) {
    when (RegNext(exception_thrown)) {
      rob_tail     := 0.U
      rob_tail_lsb := 0.U
      rob_head     := 0.U
      rob_pnr      := 0.U
      rob_pnr_lsb  := 0.U
    }
  }

  // -----------------------------------------------
  // Full/Empty Logic
  // The ROB can be completely full, but only if it did not dispatch a row in the prior cycle.
  // I.E. at least one entry will be empty when in a steady state of dispatching and committing a row each cycle.
  // TODO should we add an extra 'parity bit' onto the ROB pointers to simplify this logic?
  //  满/空逻辑
  ////ROB可以完全满，但前提是它没有在前一个周期中分派一个行。
  ////即，在每个周期处于分派和提交行的稳定状态时，至少有一个条目为空。
  ////我们应该在ROB指针上增加一个额外的奇偶校验位来简化这个逻辑吗?

  maybe_full := !rob_deq && (rob_enq || maybe_full) || io.brupdate.b1.mispredict_mask =/= 0.U
  full       := rob_tail === rob_head && maybe_full
  empty      := (rob_head === rob_tail) && (rob_head_vals.asUInt === 0.U)

  io.rob_head_idx := rob_head_idx
  io.rob_tail_idx := rob_tail_idx
  io.rob_pnr_idx  := rob_pnr_idx
  io.empty        := empty
  io.ready        := (rob_state === s_normal) && !full && !r_xcpt_val

  //-----------------------------------------------
  //-----------------------------------------------
  //-----------------------------------------------

  // ROB FSM
  if (!enableCommitMapTable) {
    switch (rob_state) {
      is (s_reset) {
        rob_state := s_normal
      }
      is (s_normal) {
        // Delay rollback 2 cycles so branch mispredictions can drain  延迟回滚2个周期，这样分支错误预测就会消耗掉
        when (RegNext(RegNext(exception_thrown))) {
          rob_state := s_rollback
        } .otherwise {
          for (w <- 0 until coreWidth) {
            when (io.enq_valids(w) && io.enq_uops(w).is_unique) {
              rob_state := s_wait_till_empty
            }
          }
        }
      }
      is (s_rollback) {
        when (empty) {
          rob_state := s_normal
        }
      }
      is (s_wait_till_empty) {
        when (RegNext(exception_thrown)) {
          rob_state := s_rollback
        } .elsewhen (empty) {
          rob_state := s_normal
        }
      }
    }
  } else {
    switch (rob_state) {
      is (s_reset) {
        rob_state := s_normal
      }
      is (s_normal) {
        when (exception_thrown) {
          ; //rob_state := s_rollback
        } .otherwise {
          for (w <- 0 until coreWidth) {
            when (io.enq_valids(w) && io.enq_uops(w).is_unique) {  //is_unique:只允许这个指令在管道中，等待STQ耗尽，在它之后清除fetch(告诉ROB未准备好直到空)
              rob_state := s_wait_till_empty
            }
          }
        }
      }
      is (s_rollback) {
        when (rob_tail_idx  === rob_head_idx) {
          rob_state := s_normal
        }
      }
      is (s_wait_till_empty) {
        when (exception_thrown) {
          ; //rob_state := s_rollback
        } .elsewhen (rob_tail === rob_head) {
          rob_state := s_normal
        }
      }
    }
  }

  // -----------------------------------------------
  // Outputs
  //告诉LSU，rob的头是一个load
  io.com_load_is_at_rob_head := RegNext(rob_head_uses_ldq(PriorityEncoder(rob_head_vals.asUInt)) &&
                                        !will_commit.reduce(_||_)) //判断load ： use_ldq



  override def toString: String = BoomCoreStringPrefix(
    "==ROB==",
    "Machine Width      : " + coreWidth,
    "Rob Entries        : " + numRobEntries,
    "Rob Rows           : " + numRobRows,
    "Rob Row size       : " + log2Ceil(numRobRows),
    "log2Ceil(coreWidth): " + log2Ceil(coreWidth),
    "FPU FFlag Ports    : " + numFpuPorts)
}
