from setuptools import find_packages, setup

package_name = 'eval_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Trajectory evaluation (ATE/RPE) + charts. PLAN eval_tools (§6).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evaluate = eval_tools.evaluate:main',
        ],
    },
)
