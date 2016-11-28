import time
import os


class TrajectoryUploader:
    """
    Class designed to upload solved trajectories
    to the rds database
    """

    trajectory_dir = "../matlab/solved_trajectories/"
    existing_trajectory_files = set()

    def load_file_to_database(self, file_path):
        """
        Takes in a file path and uploads the information
        associated with it into a postgres database
        """
        print file_path, "THIS IS THE FILE PATH"


    def load_trajectories_force(self):
        """
        Load the trajectories that we are working
        on into the cloud by force to make sure we
        aren't missing anything
        """
        # determine all of the files we have solved
        solved_files = os.listdir(self.trajectory_dir)

        # upload the info contained in the solved file
        # to a database
        for solved_file in solved_files:
            self.load_file_to_database(solved_file)

            self.existing_trajectory_files.add(solved_file)

    def watch_directory(self):
        while True:
            time.sleep(1)
        print "I AM WATCHING"
        pass


if __name__ == "__main__":
    uploader = TrajectoryUploader()

    # load all of the trajectories
    uploader.load_trajectories_force()

    # watch to see if any new ones are added to
    # the directory
    uploader.watch_directory()
