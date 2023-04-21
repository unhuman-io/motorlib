import paramiko
import datetime
import time

class RemoteClient:
    def __init__(self):
        self.uname = "figure"
        self.ip_address = "172.16.10.100"
        self.pwd = "GoFigure"

        # Create a new SSH client
        self.client = paramiko.SSHClient()
        # Automatically add the server's host key
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        try:
            self.client.connect(self.ip_address, username=self.uname, password=self.pwd,timeout=10)
        except (paramiko.AuthenticationException, paramiko.SSHException) as e:
            # If there was an authentication error or any other SSH-related error, raise an exception
            raise e

        self.sftp = self.client.open_sftp()

    def run_command(self, command):
        stdin, stdout, stderr = self.client.exec_command(command, timeout=0.1)
        exit_status = stdout.channel.recv_exit_status()          # Blocking call

        for line in iter(lambda: stdout.readline(2000000), ""):
            print(line, end="")

        for line in stdout:
            print(line.strip('\n'))
        if exit_status != 0:
            print(f'STDERR: {stderr.read().decode("utf8")}')
            print("Error", exit_status)

    def send_file(self, local_path, remote_path):
        self.sftp.put(local_path, remote_path)

    def get_file(self, remote_path, local_path):
        self.sftp.get(remote_path, local_path)

    def close_connection(self):
        client.close()