from django.core.management.base import BaseCommand, CommandError
from crazyflie.models import SolvedTrajectory
import decimal


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

    def get_match_score(self, trajectory, pitch, roll):
        """
        Takes in a trajectory and returns how close it
        is to the inputted pitch and roll
        """
        pitch_differential = abs(trajectory.pitch - pitch)
        roll_differential = abs(trajectory.roll - roll)

        return pow(pitch_differential, 2) + pow(roll_differential, 2)

    def find_closest_trajectory(self, pitch, roll):
        """
        Finds the file name associated with the
        closest trajectory
        """
        trajectories = SolvedTrajectory.objects.all()
        
        best_trajectory = None
        best_match_score = float("inf")

        for trajectory in trajectories:
            match_score = self.get_match_score(trajectory, pitch, roll)

            if match_score < best_match_score:
                best_trajectory = trajectory
                best_match_score = match_score

        return best_trajectory.file_name

    def handle(self, *args, **options):
        """
        Exposes a script to find the closest trajectory
        """

        # extract the pitch and roll
        pitch = options.get("pitch")
        roll = options.get("roll")

        # return the closest match
        return "solved_trajectories/" + self.find_closest_trajectory(pitch, roll)
