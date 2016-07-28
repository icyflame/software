from mission.framework.search import SpiralSearch
from mission.framework.helpers import ConsistencyCheck
from mission.missions.bins import BinsTask as Bins
from mission.missions.recovery import recovery as Recovery
from mission.framework.task import Task
from mission.missions.torpedoes import LocateBoard, Torpedoes
from mission.missions.hydrophones import FindPinger
from mission.framework.combinators import Sequential
from mission.framework.primitive import Log
from mission.framework.timing import Timer

import shm

class RandomPinger(Task):
    def desiredModules(self):
        if hasattr(self, "after_pinger") and hasattr(self.after_pinger, "selected_task"):
            selected_task = self.after_pinger.selected_task
            if selected_task is None or selected_task == "torpedoes":
                return [shm.vision_modules.Torpedoes]
            elif selected_task == "recovery":
                return [shm.vision_modules.Recovery]
            elif selected_task == "bins":
                return [shm.vision_modules.Bins]
        else:
            return []

    def on_first_run(self):
        self.after_pinger = AfterPinger()
        self.task = Sequential(self.after_pinger)# FindPinger(), self.after_pinger)
        self.has_made_progress = False
        # TODO @John M Update when subtask (after pinger) has made progress.

    def on_run(self):
        if self.task.finished:
            self.finish()
        else:
            self.task()

class AfterPinger(Task):
    def on_first_run(self):
        self.search = LocateBoard()
        self.torpedoes = Torpedoes()
        self.recovery = Recovery()
        self.bins = Bins()
        self.selected_task = None
        self.has_made_progress = False
        # TODO update me

    def on_run(self):
        if self.search.finished:
            if self.search.found_board:
                if not self.torpedoes.finished:
                    self.selected_task = "torpedoes"
                    self.torpedoes()
                elif not self.bins.finished:
                    self.selected_task = "bins"
                    self.bins()
                else:
                    self.finish()
            else:
                if not self.recovery.finished:
                    self.recovery()
                    self.selected_task = "recovery"
                else:
                    self.finish()
        else:
            self.search()


class AfterPingerBinsRecovery(Task):
    def recovery_validator(self):
        results = shm.recovery_vision.get()

        stacks_visible = False
        stack_visible_vars = [results.stack_1_visible, results.stack_2_visible,
                              results.stack_3_visible, results.stack_4_visible]
        num_visible_stacks = 0
        for stack_visible in stack_visible_vars:
            if stack_visible:
                num_visible_stacks += 1
        
        if num_visible_stacks >= 2:
            stacks_visible =  True 
        
        return stacks_visible or results.green_mark_visible or \
               results.red_mark_visible or results.table_visible
        
    def bins_validator(self):
        return shm.bin_cover.probability.get() > 0.0 or \
               shm.bin_yellow_1.probability.get() > 0.0 or \
               shm.bin_yellow_2.probability.get() > 0.0
         
    def check_bins_visible(self):
        print("check bins")
        if self.bins_watcher.has_changed():
            bins_val = self.bins_validator()
            print("bins validator returned", bins_val)
            self.bins_visible = self.bins_check(bins_val)
        
    def check_recovery_visible(self):
        if self.recovery_watcher.has_changed():
            self.recovery_visible = self.recovery_check(self.recovery_validator())
        
    def on_first_run(self):
        self.search = SpiralSearch(optimize_heading=True, min_spin_radius=2.0)
        self.recovery_check = ConsistencyCheck(5, 5)
        self.bins_check = ConsistencyCheck(5, 5)

        self.bins_watcher = shm.watchers.watcher()
        self.bins_watcher.watch(shm.bin_cover)
        self.bins_watcher.watch(shm.bin_yellow_1)
        self.bins_watcher.watch(shm.bin_yellow_2)
        self.bins_visible = False

        self.recovery_watcher = shm.watchers.watcher()
        self.recovery_watcher.watch(shm.recovery_vision)
        self.recovery_visible = False

        self.bins = Bins()
        self.recovery = Recovery()

        self.task = None

    def on_run(self):
        self.check_recovery_visible()
        self.check_bins_visible()
     
        print("bins visible is", self.bins_visible)
     
        if self.task is None:
            if self.bins_visible:
                self.logi("Found bins!")
                self.task = self.bins
            elif self.recovery_visible:
                self.logi("Found recovery!")
                self.task = self.recovery
            else:
                self.search()
        else:
            self.task()
            
random_pinger = RandomPinger()
after_pinger = AfterPinger()
