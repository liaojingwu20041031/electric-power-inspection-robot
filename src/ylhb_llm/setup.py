from glob import glob
import os
from setuptools import setup

package_name = 'ylhb_llm'


def qml_data_files():
    entries = []
    for root, _dirs, files in os.walk('qml'):
        source_files = [os.path.join(root, name) for name in files]
        if source_files:
            entries.append((os.path.join('share', package_name, root), source_files))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ] + qml_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liaojingwu20041031',
    maintainer_email='206929594+liaojingwu20041031@users.noreply.github.com',
    description='Electric power inspection robot AI task layer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspection_task_node = ylhb_llm.inspection_task_node:main',
            'inspection_agent_node = ylhb_llm.inspection_agent_node:main',
            'inspection_display_ui_node = ylhb_llm.inspection_display_ui_node:main',
            'basic_motion_command_node = ylhb_llm.basic_motion_command_node:main',
            'base_motion_skill_node = ylhb_llm.base_motion_skill_node:main',
            'voice_input_node = ylhb_llm.voice_input_node:main',
            'voice_session_node = ylhb_llm.voice_session_node:main',
            'voice_output_node = ylhb_llm.voice_output_node:main',
            'system_supervisor_node = ylhb_llm.system_supervisor_node:main',
            'check_agent_setup = ylhb_llm.check_agent_setup:main',
        ],
    },
)
