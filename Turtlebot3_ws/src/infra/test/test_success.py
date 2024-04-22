import pytest
import subprocess
import time

@pytest.fixture(scope="session")
def ros_launch(request):
    process = subprocess.Popen(['ros2', 'launch', 'infra', 'initialization_success.launch.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    yield process
    process.terminate()
    process.wait()

def test_check_output(ros_launch):
    msg = 'Goal succeeded'
    time.sleep(200)  # Adjust the sleep time as needed
    stdout, stderr = ros_launch.communicate()
    assert msg in stdout, f"Expected '{msg}' not found in the output."
