# Base Inertial Parameter Identification - Examples
## Franka
### Real test
`franka_real` - estimation using real data (/rosbag/franka_real_fourier_CT')

### Estimation of SoftHand (sim)
`franka_sim_softhand` - Estimation of the last link (Link7 + SoftHand) (rosbag/softhand_lean_fourier_CT)
### Test with different exciting trajectories (sim)
`franka_sim_trajectories` (./rosbag/softhand_chirp_CT, ./rosbag/softhand_lean_fourier_CT ,./rosbag/softhand_sinusoidal_CT)

## Allegro Hand
`allegro_hand`
### Computation of finger hand model
`ahand_finger` (allegro_hand_4traject_bag), `ahand_thumb` (allegro_hand_thumb_2traject_bag)