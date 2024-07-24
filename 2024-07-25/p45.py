# Import bluesky and ophyd
import bluesky.plans as bp
import bluesky.preprocessors as bpp
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky.utils import ProgressBarManager, register_transform
from dodal.beamlines import p45
from dodal.common.beamlines.beamline_utils import set_directory_provider

from ophyd_async.core import (
    HardwareTriggeredFlyable,
    AutoIncrementFilenameProvider,
    PathProvider,
    StaticFilenameProvider,
    StaticPathProvider,
    TriggerInfo,
    DetectorTrigger,
)
from ophyd_async.panda import (
    StaticPcompTriggerLogic,
    HDFPanda,
    PcompInfo,
    PcompDirectionOptions,
)
from ophyd_async.epics.motion.motor import Motor, FlyMotorInfo
from ophyd_async.epics.pmac import PmacTrajectory, FlyTrajectoryInfo
from ophyd_async.plan_stubs import ensure_connected, fly_and_collect
from scanspec.specs import Line, fly


# Create a run engine, with plotting, progressbar and transform
RE = RunEngine({}, call_returns_result=True)
RE.waiting_hook = ProgressBarManager()
# register_transform("RE", prefix="<")

# Create ophyd-async devices
# fp = StaticFilenameProvider("data")
auto_inc_fp = AutoIncrementFilenameProvider(inc_delimeter="")
set_directory_provider(StaticPathProvider(auto_inc_fp, "/dls/p45/data/2024/cm37283-2/tmp"))
det = p45.diff()
panda: HDFPanda = p45.panda1()
motor = Motor("BL45P-MO-STAGE-01:CS:Y", "CS Y")
prefix = "BL45P-MO-STEP-01"
traj = PmacTrajectory(prefix, "BRICK1.CS3", motor, name="sim_pmac")
pcomp_flyer = HardwareTriggeredFlyable(StaticPcompTriggerLogic(panda.pcomp[1]))
dets = [det, panda]
# A margin of safety based on the jitteryness of the motor
deadtime = 0.05


@bpp.stage_decorator(dets)
@bpp.run_decorator()
def my_custom_plan(start_position=0, end_position=2, exposure=0.1, nframes=100):
    yield from ensure_connected(motor)
    yield from ensure_connected(traj)
    # Put a dataset in, this will eventually be captured by load/save
    # This should be done before stage, so might have to run this twice

    # TODO: should come from motor controller
    resolution = yield from bps.rd(panda.inenc[3].val_scale)

    def in_cts(val: float) -> int:
        return int(val / resolution)

    direction = (
        PcompDirectionOptions.positive
        if in_cts(end_position - start_position) > 0
        else PcompDirectionOptions.negative
    )
    # spec = fly(Line(motor, start_position, end_position, nframes), exposure)
    # stack = spec.calculate()
    print("Before prepares")
    yield from bps.prepare(
        traj,
        FlyTrajectoryInfo(
            start_position=start_position,
            end_position=end_position,
            num_positions=nframes,
            time_per_position=exposure,
        ),
    )
    # CS Y Stretch is set to 0.5 so it's encoder reads 0.25 more
    yield from bps.prepare(
        pcomp_flyer,
        PcompInfo(
            start_postion=in_cts(start_position),
            pulse_width=1,
            rising_edge_step=abs(in_cts((end_position - start_position) / nframes)),
            number_of_pulses=nframes,
            direction=direction,
        ),
    )
    for d in dets:
        yield from bps.prepare(
            d,
            TriggerInfo(
                number=nframes,
                trigger=DetectorTrigger.constant_gate,
                livetime=exposure,
                deadtime=deadtime,
            ),
        )
    print("Wait for prepares")
    yield from bps.wait()
    print("Fly and collect")
    yield from fly_and_collect(
        leader_flyer=traj,
        detectors=dets,
        follower_flyers=[pcomp_flyer],
        stream_name="primary",
    )
    # print("Declare stream")
    # yield from bps.declare_stream(*dets, name="primary", collect=True)
    # print("Kickoff all")
    # yield from bps.kickoff_all(motor, pcomp_flyer, *dets)
    # print("Collect while completing")
    # yield from bps.collect_while_completing([motor, pcomp_flyer], dets, flush_period=0.5, stream_name="primary")

RE(bps.abs_set(panda.inenc[3].val_dataset, "y-rbv"))
RE(my_custom_plan())
