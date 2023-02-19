![SMC](https://github.com/khulqu15/smc_drone/blob/master/matlab/image/drone.jpg?raw=true)

# SMC VS SMC-KF
Perbedaan respon antara sliding mode control (SMC) dan sliding mode control dengan filter Kalman (SMC-KF) dapat diamati pada estimasi posisi dan kecepatan.

## SMC

Dalam kasus SMC, masukan kontrol dihitung hanya berdasarkan kesalahan antara posisi dan kecepatan yang diinginkan dan perkiraan keadaan saat ini. Input kontrol diperbarui pada setiap langkah waktu berdasarkan tanda kesalahan. Ini menghasilkan algoritme kontrol yang relatif sederhana, tetapi responsnya mungkin kurang akurat karena kurangnya penyaringan kebisingan pengukuran.

### Respon

![SMC](https://github.com/khulqu15/smc_drone/blob/master/matlab/image/smc.jpg?raw=true)

## SMC-KF

Dalam kasus SMC-KF, filter Kalman digunakan untuk memperkirakan keadaan sistem dengan menggabungkan pengukuran derau dengan model dinamis sistem. Hal ini menghasilkan estimasi posisi dan kecepatan yang lebih akurat, karena kebisingan pengukuran disaring oleh filter Kalman.

![SMCKF](https://github.com/khulqu15/smc_drone/blob/master/matlab/image/smckf.jpg?raw=true)

Perbedaan respon dapat dilihat dengan membandingkan plot estimasi posisi dan kecepatan untuk algoritma SMC dan SMC-KF. Algoritma SMC-KF akan menghasilkan perkiraan posisi dan kecepatan yang lebih halus dan lebih akurat, dibandingkan dengan algoritma SMC, yang mungkin menunjukkan lebih banyak fluktuasi dan kesalahan karena kebisingan pengukuran.