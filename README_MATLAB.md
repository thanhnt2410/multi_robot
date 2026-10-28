# Mo phong Kuramoto dao thoi gian va duong Lissajous tren MATLAB

**Cách chạy:** mở thư mục này trong MATLAB và nhập
`results = run_paper_simulation;`. Mã chính không cần toolbox bổ sung.
Chạy `test_paper_simulation` để kiểm tra và
`animate_paper_simulation(results.detection,5)` để xem chuyển động.

**Đã kiểm tra trên MATLAB R2025a:** mô hình lý tưởng phát hiện đủ 10.000
mục tiêu, phủ 100% lưới; kiểm tra cân bằng quay, khôi phục sau nhiễu,
dừng đúng thời gian và độ hội tụ theo bước tích phân đều đạt.
MPCC đã được kiểm tra khởi chạy trong 0,2 giây mô phỏng (6 lần gọi bộ
giải, tất cả có exitflag dương); chưa xác nhận toàn bộ thí nghiệm MPCC
30 giây. Xem nhật ký `matlab_validation.log`.

Tai lieu nguon: Boldrer et al., *Time-Inverted Kuramoto Model Meets Lissajous
Curves: Multi-Robot Persistent Monitoring and Target Detection*, IEEE RA-L,
8(1), 240-247, 2023, DOI 10.1109/LRA.2022.3224661.

## Chay nhanh

Mo MATLAB, chuyen Current Folder den thu muc chua file nay:

```matlab
results = run_paper_simulation;
test_paper_simulation;
animate_paper_simulation(results.detection, 5);
```

Ban mo phong chinh chi can MATLAB co ban (khuyen nghi R2020b tro len).
Khong can Simulink, Robotics System Toolbox, hay Optimization Toolbox.
Chay khong hien cua so:

```matlab
results = run_paper_simulation(struct('visible',false));
```

Hinh PNG, hinh MATLAB FIG va du lieu MAT duoc luu trong `results/`.
Neu khong muon ghi file: `struct('saveFigures',false)`.

## Cac kich ban

1. **Detection**: N=15, omega=0.05 rad/s, 10.000 muc tieu di dong.
   Gioi han ly thuyet T_max=8.37758 s, tuong ung noi dung Hinh 2-3.
   Ve quy dao, dia cam bien, so muc tieu da phat hien va do phu luoi.
   Kiem tra ca do phu cua rieng robot 1; khoang thoi gian ngan khong nhat
   thiet du de tung robot hoan thanh mot vong.
2. **Failure**: N=5, a=3, b=2, A=1.8 m, B=0.8 m, omega=0.3 rad/s.
   Vong truyen thong 1-2-5-3-4-1 dung Sec. VII. Robot 1 va 5 dung tu
   9.6 den 19.5 s; thoi gian lay tu Hinh 6, hinh hoc lay tu thi nghiem
   Sec. VII. So sanh phan hoi Kuramoto va baseline dot(theta)=omega.
3. **Perturbation**: N=10, a=7, b=3, A=B=5 m theo Hinh 4.
   Chon p=3; robot 1 bi lech 0.12 rad tai t=5 s. Theo doi phuc hoi,
   dieu kien cos(delta)<0 va khoang cach nho nhat.

Tat ca gia tri co the sua trong `+tikm/config.m`. Hoac chay rieng:

```matlab
c = tikm.config('failure');
c.failureWindow = [5 8];
c.T = 35;
s = tikm.simulate(c);
plot(s.t, s.theta); grid on;
```

## Phuong trinh va cach hieu pha

Ma dung dung dau tru cua Eq. (1), khong chia tong tuong tac cho bac do thi:

    dot(theta_i) = omega - sum_j sin(theta_j-theta_i).

Moi robot co hai hang xom trong vong vo huong. Pha duoc luu khong wrap
de tranh gian doan tai 2*pi. Chi wrap khi tinh sai so tuong doi.
Vi tri vat ly la `x=A*cos(a*theta), y=B*sin(b*theta)` (Eq. 3).
Khong dung truc tiep theta trong Eq. (4) nhu tham so quy dao: Eq. (4)
la bieu dien hinh hoc cua ho ellipse tai can bang.

Can bang quay duoc khoi tao theo thu tu vong truyen thong:

    theta(order(k)) = 2*pi*p*(k-1)/N.

Chon p nguyen trong (N/4,3*N/4); kappa=gcd(N,p).
Kappa=1 nghia la N pha khac nhau, moi nhom dong pha chi co mot robot.
Sai so doi hinh loai bo goc quay chung, do do khong nham sai lech
pha chung voi mat doi hinh.

Tich phan RK4 voi buoc 0.01 s, chen chinh xac cac moc su co/nhieu.
Ma tinh cac gioi han Eq. (6), (7), (8) va T_max, dong thoi do khoang
cach Euclid nho nhat giua MOI cap robot. Khong ep tranh va cham trong
qua do. Dieu kien Eq. (6) chi ap dung cho can bang 1-clustered phu hop.

Muc tieu duoc khoi tao ngau nhien voi seed 42, toc do 0.2-1.0 m/s,
phan xa tai bien chu nhat. Phat hien la phep do hoan hao trong dia
cam bien. Do phu duoc danh gia tren luoi 101x81. Cac phep do lay mau
khong thay the chung minh lien tuc; giam dt va tang mat do luoi de
danh gia do nhay so.

## Mo phong xe voi MPCC (tuy chon)

```matlab
% Can Optimization Toolbox; moi xe giai mot bai toan fmincon moi buoc.
car = run_mpcc_demo(struct('T',8));
% Thi nghiem su co day du hon; co the chay lau:
car = run_mpcc_demo(struct('T',30,'failureWindow',[9.6 19.5]));
```

3 xe mo phong vat ly la robot 2,3,4; robot ao la 1,5. Khong co ket
noi phan cung. Dung mo hinh xe dap Eq. (12), trong so, gioi han dau vao
va M=15 nhu bai bao. Theo doi toc do pha bang chi phi Eq. (15), giu
lenh pha co dinh trong moi chan troi du bao. Khong co rang buoc tranh
va cham, tuong tu lua chon cua tac gia. Kiem tra `solverExitFlag`,
`minDistance` va `localPathDistance`; khong coi ket qua toi uu la bao
dam an toan. Buoc lay mau mac dinh 0.1 s la lua chon trien khai.

Day la hien thuc MPCC tham khao, khong tai tao tung chi tiet bo dieu
khien cua tac gia: gamma(s) dung noi suy pchip thay vi da thuc; chi phi
dung sai so tiep tuyen/phap tuyen tai diem tien do; khong co du lieu
nhieu phan cung, khoi tao do thuc, ma giai toi uu hay chu ky dieu
khien goc. Do do khong ky vong lap lai chinh xac Hinh 7-9 hay he so
1.2429. Khoang cach den duong la tim cuc tieu cuc bo quanh pha hien tai.

## Pham vi tai lap

Tai lap phuong trinh, cac co che va thu nghiem so tuong ung; khong
so hoa chinh xac moi hinh. Bai bao khong cong bo day du vi tri va
quy dao 10.000 muc tieu, seed, va moi tham so cua Hinh 2-6.
Hinh hoc a=7,b=8,A=B=5 trong detection la lua chon minh hoa; khong
gan nhan la gia tri tac gia. Khong ep thoi diem phat hien thanh 4.5 s.
Ban nay khong tuyen bo mo phong duoc UAV 3D, vat can, mat lien lac,
hoac hong vinh vien. Mo rong Lissajous 3D can kiem tra giao cat va
khoang cach than robot rieng, khong chi kiem tra tan so nguyen to cung nhau.
