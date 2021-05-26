import time
import math
import rclpy
import os
from scipy.special import erfinv
from rclpy.node import Node
from rcl_interfaces.msg import TrafficModel
import multiprocessing
import vpl

import atexit

import logging
logger = logging.getLogger('Preemption Controller')
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s: %(message)s'))
logger.addHandler(ch)

machine_id = os.getenv("ROS_MACHINE_ID")

class Solver(Node):
    def __init__(self, plans, start_times, lock):
        super().__init__('solver'+machine_id)
        self.sub = self.create_subscription(
            TrafficModel, "ros_traffic_model_"+machine_id, self.on_receive_model, 10)
        self.plans = plans
        self.start_times = start_times
        self.lock = lock

    def solve(self, model):
        # solve delta_b from prob
        delta_b = math.sqrt(2)*model.sigma_t*erfinv(0.95)

        p = model.a
        q = model.b-delta_b
        r = delta_b + delta_b + 2e-3  # early relaxing is killing, so leave a margin

        return p, q, r

    def on_receive_model(self, model):
        logger.info(
            f'new traffic model of {model.id}: {model.a} {model.b} {model.sigma_t}')
        if model.sigma_t > 5e-3:
            logger.warn('too large sigma_t, ignore')
            return

        p, q, r = self.solve(model)
        logger.info(f'plan for {model.id}: {p} {q} {r}')

        self.lock.acquire()

        # update/add new plan
        self.plans[model.id] = (p, q, r)
        # also compute the next time, may overwrite existing one
        t = time.monotonic()
        self.start_times[model.id] = math.ceil((t-q)/p)*p+q

        self.lock.release()


def sleep_until(time_wakeup):
    tts = time_wakeup-time.monotonic()-0.8e-3
    if tts <= 0:
        return
    time.sleep(tts)


def inform_queue_manager(t):
    try:
        vpl.valve_startprotect(int(t*1e9))
    except OSError:
        logger.error('vpl not ok')


def inform_and_relax_queue_manager(t):
    try:
        vpl.valve_renewprotect(int(t*1e9))
    except OSError:
        logger.error('vpl not ok')


def relax_queue_manager():
    try:
        vpl.valve_endprotect()
    except OSError:
        logger.error('vpl not ok')


def get_time():
    return time.monotonic()


def run_executor(plans, start_times, lock):
    relax_queue_manager()
    informed = False 
    t_end = None
    while True:
        if len(plans) == 0:
            time.sleep(1)
            continue
        # compute the type and time of LOGICAL next event
        lock.acquire()
        # the time of next 'start'
        idx, t_start = min(start_times.items(), key=lambda x: x[1])
        p, _, r = plans[idx]
        # the time of 'end' corresponding to this 'start'
        this_t_end = t_start + r
        # the time of next 'end'
        # two cases:
        # 1. t_end is None, then next 'end' event should be exactly corresponding to this 'start' event
        # 2. t_end is not None, then if it is earlier than this 'start' event, nothing to do with t_end;
        #    if it is later than this 'start', it means we have started a protection, and should put off
        #    the time of ending protection if necessary
        if t_end is None:
            t_end = this_t_end
        elif t_end > t_start:
            t_end = max(t_end, this_t_end)

        # process the event
        # if 'start', inform the start time and sleep
        # if 'end', sleep until that time, inform the next start time, then stop this protection
        if t_start <= t_end:
            start_times[idx] += p
            lock.release()
            if get_time() < t_start:
                if not informed:
                    inform_queue_manager(t_start)
                    informed = True
                    logger.debug(f'inform {t_start} for flow {idx}')
                sleep_until(t_start)
                logger.debug(f'enter protection range for flow {idx}, {get_time()}')
            else: # we are late
                if not informed:
                    logger.debug(f'too late to inform, skip')
        else:  # if t_start > t_end
            lock.release()
            if get_time() < t_end:
                sleep_until(t_end)
                # inform before relax to avoid dequeuing too many packets which may hurt the next transmission
                # we'll often double-inform, but it's not a problem
                inform_and_relax_queue_manager(t_start)
                informed = True
                logger.debug(f'inform {t_start} for flow {idx}')

                logger.debug(f'relax at {get_time()}')
            else:
                logger.debug(f'too late to relax, but still end')
                # relax anyway
                relax_queue_manager()
                informed = False
            t_end = None


def run_solver(plans, start_times, lock):
    rclpy.init()
    node = Solver(plans, start_times, lock)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    atexit.register(relax_queue_manager)

    plans = multiprocessing.Manager().dict()
    start_times = multiprocessing.Manager().dict()
    lock = multiprocessing.Lock()

    p = []
    p.append(multiprocessing.Process(target=run_solver, args=(plans, start_times, lock)))
    p.append(multiprocessing.Process(target=run_executor, args=(plans, start_times, lock)))
    for q in p:
        q.start()
    for q in p:
        q.join()
