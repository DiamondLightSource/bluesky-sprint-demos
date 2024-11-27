# Import bluesky and ophyd
import bluesky.plans as bp
import bluesky.preprocessors as bpp
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky.utils import ProgressBarManager, register_transform
from dodal.beamlines import p99
from dodal.common.beamlines.beamline_utils import set_path_provider

from ophyd_async.core import (
    StandardFlyer,
    AutoIncrementFilenameProvider,
    PathProvider,
    StaticFilenameProvider,
    StaticPathProvider,
    TriggerInfo,
    DetectorTrigger,
    StandardFlyer
)
from ophyd_async.fastcs.panda import (
    HDFPanda,
    PcompInfo,
    ScanSpecInfo,
    ScanSpecSeqTableTriggerLogic,
)
from ophyd_async.epics.pmac import Pmac, PmacMotor,PmacTrajectoryTriggerLogic, PmacTrajInfo
from ophyd_async.plan_stubs import ensure_connected, fly_and_collect

from scanspec.specs import Line, fly


# Create a run engine, with plotting, progressbar and transform
RE = RunEngine({}, call_returns_result=True)
RE.waiting_hook = ProgressBarManager()
# register_transform("RE", prefix="<")

# Create ophyd-async devices
# fp = StaticFilenameProvider("data")
auto_inc_fp = AutoIncrementFilenameProvider(inc_delimeter="")
set_path_provider(StaticPathProvider(auto_inc_fp, "/dls/p99/data/2024/cm37284-5/tmp"))
# det = p45.diff()
panda: HDFPanda = p99.panda1()
y_motor = PmacMotor("BL99P-MO-STAGE-02:Y")
x_motor = PmacMotor("BL99P-MO-STAGE-02:X")
prefix = "BL99P-MO-STEP-01"
pmac = Pmac(prefix, name="p99_pmac")

# pcomp_flyer = HardwareTriggeredFlyable(StaticPcompTriggerLogic(panda.pcomp[1]))
dets = [panda]
# A margin of safety based on the jitteryness of the motor
deadtime = 0.001


@bpp.stage_decorator(dets)
@bpp.run_decorator()
def my_custom_plan(start_fast_position=1, end_fast_position=5, start_slow_position=1, end_slow_position=2, exposure=0.5, nfastframes=10, nslowframes=10):

    yield from ensure_connected(x_motor)
    yield from ensure_connected(y_motor)
    yield from ensure_connected(pmac)
    # Put a dataset in, this will eventually be captured by load/save
    # This should be done before stage, so might have to run this twice

    # TODO: should come from motor controller
    resolution = yield from bps.rd(panda.inenc[3].val_scale)

    def in_cts(val: float) -> int:
        return int(val / resolution)

    # direction = (
    #     PcompDirectionOptions.positive
    #     if in_cts(end_position - start_position) > 0
    #     else PcompDirectionOptions.negative
    # )
    # spec = fly(Line(motor, start_position, end_position, nframes), exposure)
    # stack = spec.calculate()
    print("Before prepares")

    # y_motor = Motor("BL45P-MO-STAGE-01:CS:Y")
    # x_motor = Motor("BL45P-MO-STAGE-01:X")
        # panda = Panda("BL45P-MO-PANDA-01:")
    spec = fly(Line(y_motor, start_slow_position, end_slow_position, nslowframes) * ~Line(x_motor, start_fast_position, end_fast_position, nfastframes), exposure)
    traj=PmacTrajectoryTriggerLogic(pmac)
    panda_seq=ScanSpecSeqTableTriggerLogic(panda.seq[1])
    traj_flyer=StandardFlyer(traj)
    pandaseq_flyer=StandardFlyer(panda_seq)
    # seq_info = ScanSpecInfo(spec=spec, deadtime=0.1)
    # seq_trigger_logic = ScanSpecSeqTableTriggerLogic(mock_panda.seq[1])
    # pmac_info = PmacTrajInfo(spec=spec)
    # pmac_trigger_logic = PmacTrajectoryTriggerLogic(pmac)
    # await trigger_logic.prepare(seq_info)
    # await pmac_trigger_logic.prepare(pmac_info)
    # print("Before PMAC PREPARE")
    yield from bps.prepare(
        traj_flyer,
        PmacTrajInfo(
            spec=spec
        ),
        wait=False,
        group="prep"
    )
    print("Before PandA PREPARE")
    yield from bps.prepare(
        pandaseq_flyer,
        ScanSpecInfo(spec=spec, deadtime=0.001),
    )
    # CS Y Stretch is set to 0.5 so it's encoder reads 0.25 more
    # yield from bps.prepare(
    #     pcomp_flyer,
    #     PcompInfo(
    #         start_postion=in_cts(start_fast_position),
    #         pulse_width=1,
    #         rising_edge_step=abs(in_cts((end_fast_position - start_fast_position) / nfastframes)),
    #         number_of_pulses=nfastframes,
    #         direction=direction,
    #     ),
    # )
    # print("Before Det PREPARE")
    # yield from bps.prepare(
    #     panda_seq,
    #     TriggerInfo(
    #         number_of_triggers=nfastframes*nslowframes,
    #         trigger=DetectorTrigger.constant_gate,
    #         livetime=exposure,
    #         deadtime=deadtime,
    #     ),
    #     group="prep"
    # )

    for d in dets:
        yield from bps.prepare(
            d,
            TriggerInfo(
                number_of_triggers=nfastframes*nslowframes,
                trigger=DetectorTrigger.constant_gate,
                livetime=exposure,
                deadtime=deadtime,
            ),
            group="prep"

        )
    print("Wait for prepares")
    yield from bps.wait(group="prep")
    print("Fly and collect")
    yield from fly_and_collect(
        flyers=[traj_flyer, pandaseq_flyer],
        stream_name="primary",
        detectors=dets
    )
    # yield from fly_and_collect(
    #     flyer=pandaseq_flyer,
    #     stream_name="primary",
    #     detectors=dets
    # )
    print("Declare stream")
    yield from bps.declare_stream(*dets, name="primary", collect=True)
    print("Kickoff all")
    # yield from bps.kickoff_all(traj_flyer, pandaseq_flyer, *dets)
    # print("Collect while completing")
    yield from bps.collect_while_completing([traj_flyer, pandaseq_flyer], dets, flush_period=0.5, stream_name="primary")

RE(bps.abs_set(panda.inenc[2].val_dataset, "x-rbv"))
RE(bps.abs_set(panda.inenc[3].val_dataset, "y-rbv"))
RE(my_custom_plan())
