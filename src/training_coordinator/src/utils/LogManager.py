import os
import shutil
from datetime import datetime

class LogManager:
    def __init__(self, log_container_path: str):
        self.log_container_path = log_container_path

    def keep_last_x_logs(self, num_logs_to_keep: int):
        if not os.path.exists(self.log_container_path):
            return

        contents = os.listdir(self.log_container_path)
        dirs = [content for content in contents if os.path.isdir(os.path.join(self.log_container_path, content))]
        dirs.sort()
        dirs_to_delete = dirs[:-num_logs_to_keep]
        print(dirs_to_delete)
        for dir in dirs_to_delete:
            print(dir)
            shutil.rmtree(os.path.join(self.log_container_path, dir))

    def create_next_log_path(self):
        now = datetime.now()
        date_time_str = now.strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.log_container_path, f"{date_time_str}_training")