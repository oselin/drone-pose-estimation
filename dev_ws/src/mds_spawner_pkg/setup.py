from setuptools import setup
from glob import glob
package_name = 'mds_spawner_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name, glob('launch/*.py')),
        ('share/'+ package_name, glob('iris/*.urdf')),
        ('share/'+ package_name, glob('iris/meshes/*.dae')),
        ('share/'+ package_name, glob('./worlds/*.world')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giacomo',
    maintainer_email='giacomo.mutti@studenti.unitn.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mds_spawner = mds_spawner_pkg.mds_spawner:main'
        ],
    },
)



#(os.path.join('share', package_name,'models/'), glob('./worlds/*')),