from django.core.management.base import BaseCommand, CommandError
from crazyflie.models import SolvedTrajectory
import decimal

import time
start = time.time()

class Command(BaseCommand):
    help = 'Starts a process to sync the solved trajectories to the database'

    def add_arguments(self, parser):
        """
        Accept the pitch and roll from the command line
        """
        parser.add_argument(
            '--pitch',
            required=True,
            type=decimal.Decimal)

        parser.add_argument(
            '--roll',
            required=True,
            type=decimal.Decimal)

        parser.add_argument(
            '--min_roll_differential',
            default=decimal.Decimal(.05),
            type=decimal.Decimal)

        parser.add_argument(
            '--ideal_min_roll_differential',
            default=decimal.Decimal(.05),
            type=decimal.Decimal)

        parser.add_argument(
            '--min_pitch_differential',
            default=decimal.Decimal(.01),
            type=decimal.Decimal)

        parser.add_argument(
            '--ideal_min_pitch_differential',
            default=decimal.Decimal(.01),
            type=decimal.Decimal)

    def get_match_score(self, trajectory, pitch, roll):
        """
        Takes in a trajectory and returns how close it
        is to the inputted pitch and roll
        """
        pitch_differential = abs(trajectory.pitch - pitch)
        roll_differential = abs(trajectory.roll - roll)

        return pow(pitch_differential, 2) + pow(roll_differential, 2)

    def find_closest_trajectory(self, **kwargs):
        """
        Finds the file name associated with the
        closest trajectory
        """
        # determine bounds on the pitch and the roll
        # of the trajectory we will return
        min_pitch = kwargs["pitch"] - kwargs["min_pitch_differential"]
        max_pitch = kwargs["pitch"] + kwargs["min_pitch_differential"]

        min_roll = kwargs["roll"] - kwargs["min_roll_differential"]
        max_roll = kwargs["roll"] + kwargs["min_roll_differential"]

        print time.time() - start, "SST"
        # determine the candidate trajectories
        candidate_trajectories = SolvedTrajectory.objects.filter(
            pitch__gt=min_pitch,
            roll__gt=min_roll
        ).filter(
            pitch__lt=max_pitch,
            roll__lt=max_roll
        )
        print time.time() - start, "SST2"
        
        # if we can find an approximation that works to two
        # decimal places, just return that
        ideal_min_pitch = kwargs["pitch"] - kwargs["ideal_min_pitch_differential"]
        ideal_max_pitch = kwargs["pitch"] + kwargs["ideal_min_pitch_differential"]

        ideal_min_roll = kwargs["roll"] - kwargs["ideal_min_roll_differential"]
        ideal_max_roll = kwargs["roll"] + kwargs["ideal_min_roll_differential"]

        # find trajectories that we are good with even if they aren't the absolute
        # best
        ideal_trajectory = SolvedTrajectory.objects.filter(
            pitch__gt=ideal_min_pitch,
            roll__gt=ideal_min_roll
        ).filter(
            pitch__lt=ideal_max_pitch,
            roll__lt=ideal_max_roll)
        print ideal_trajectory.query
        ideal_trajectory = ideal_trajectory.first()
        print time.time() - start, "SST3"

        # if we found something in the ideal trajectory, just return that!
        if ideal_trajectory:
            print "FOUND IDEAL"
            return ideal_trajectory.file_name

        # otherwise, filter through and find the best match
        best_trajectory = None
        best_match_score = float("inf")

        for trajectory in candidate_trajectories:
            match_score = self.get_match_score(trajectory, pitch, roll)

            if match_score < best_match_score:
                best_trajectory = trajectory
                best_match_score = match_score

        return best_trajectory.file_name

    def handle(self, *args, **options):
        """
        Exposes a script to find the closest trajectory
        """

        # return the closest match
        res = "solved_trajectories/" + self.find_closest_trajectory(**options)
        print time.time() - start
        return res
