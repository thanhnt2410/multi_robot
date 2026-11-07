#!/usr/bin/env python3
"""Publish the measured Gazebo test outcomes and figures into validation/."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from run_gazebo_suite import analyze


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite',type=Path)
    parser.add_argument('--faults',type=Path)
    parser.add_argument('--five-rerun',type=Path)
    parser.add_argument('--hybrid-rerun',type=Path)
    parser.add_argument('--baseline-rerun',type=Path)
    parser.add_argument('--perturbation-rerun',type=Path)
    args=parser.parse_args();root=Path(__file__).resolve().parent.parent
    target=root/'validation/gazebo_suite';target.mkdir(parents=True,exist_ok=True)
    results=json.loads((args.suite/'suite_summary.json').read_text())
    sources={name:args.suite/name for name in results}
    historical={}
    for name,replacement in [('five_physical',args.five_rerun),('failure_recovery',args.hybrid_rerun),
                             ('open_loop_failure',args.baseline_rerun),('phase_perturbation',args.perturbation_rerun)]:
        if replacement:
            historical[name]=results[name]
            (target/f'{name}_before_fix.json').write_text(json.dumps(results[name],indent=2)+'\n')
            results[name]=analyze(replacement,name);sources[name]=replacement
    comparison=json.loads((args.suite/'comparison.json').read_text())
    if args.hybrid_rerun and args.baseline_rerun:
        feedback=results['failure_recovery']['summary']['formation_final']
        baseline=results['open_loop_failure']['summary']['formation_final']
        comparison=dict(feedback_final_formation=feedback,open_loop_final_formation=baseline,
                        feedback_improves_formation=feedback<baseline)
    labels={'normal':'Tuần tra bình thường (3+2)', 'failure_recovery':'Dừng và phục hồi (3+2)',
            'open_loop_failure':'Vòng hở khi dừng tác tử (3+2)',
            'phase_perturbation':'Nhiễu pha 0,12 rad (3+2)', 'five_physical':'Năm xe vật lý, dừng và phục hồi'}
    lines=['# Kiểm thử các kịch bản trên Gazebo Fortress','',
           'Các lượt dưới đây chạy vật lý Gazebo headless, tuần tự để hạn chế ảnh hưởng của tranh chấp CPU. '
           'Mỗi kịch bản nhiệm vụ chạy 180 giây mô phỏng, seed 42, 10.000 mục tiêu với sensing đĩa lý tưởng.','',
           '| Kịch bản | Kết quả | RMS đường [m] | Max đường [m] | Min khoảng cách [m] | Contact robot | Lỗi cập nhật | Pha cuối RMS [rad] |',
           '|---|---|---:|---:|---:|---:|---:|---:|']
    for name,result in results.items():
        (target/f'{name}.json').write_text(json.dumps(result,indent=2)+'\n')
        metadata=sources[name]/'metadata.json'
        if metadata.exists():shutil.copy(metadata,target/f'{name}_metadata.json')
        config=args.suite/'scenarios'/f'{name}.yaml'
        if config.exists():shutil.copy(config,target/f'{name}.yaml')
        summary=result.get('summary',{})
        if summary:
            lines.append(f'| {labels.get(name,name)} | {"ĐẠT" if result["passed"] else "CHƯA ĐẠT"} '
                         f'| {summary["tracking_rms"]:.4f} | {summary["tracking_max"]:.4f} '
                         f'| {summary["min_pair_distance"]:.3f} | {summary["robot_contact_samples"]} '
                         f'| {summary["failed_control_updates"]} | {summary["formation_final"]:.5f} |')
            subprocess.run([sys.executable,str(root/'scripts/plot_run.py'),str(sources[name]),
                            '--output',str(target/f'{name}.png')],check=True)
    lines+=['','## Tiêu chí và sự kiện kiểm tra','',
            'Đặt tiêu chí trước khi chạy: hoàn tất 180 s, không tiếp xúc giữa robot, khoảng cách mọi cặp '
            'trên 0,84 m (đường kính bao xe), RMS bám đường dưới 0,15 m, sai số lớn nhất dưới 0,5 m, '
            'dưới 5% cập nhật điều khiển thất bại, phủ lưới toàn đội và phát hiện đủ mục tiêu lý tưởng. '
            'Các ca có phản hồi cần sai số pha cuối dưới 0,03 rad. Ca vòng hở cần quan sát được suy giảm đội hình trên 0,05 rad.','',
            'Ca dừng/phục hồi kiểm tra tác tử 1 và 5 dừng trong [38,4; 78] s và tiến tiếp sau 120 s. '
            'Đội có phản hồi được kiểm tra vận tốc trung bình dưới 0,05 m/s trong đoạn cuối sự cố. '
            'Ca nhiễu pha không có sự cố dừng; tác tử ảo 1 nhận bước pha 0,12 rad tại 60 s, '
            'được xác nhận bằng bước nhảy pha trong dữ liệu.','']
    for name,result in results.items():
        failed=[k for k,v in result.get('checks',{}).items() if not v]
        lines.append(f'- **{labels.get(name,name)}:** '+('các kiểm tra đạt.' if not failed and result.get('passed') else 'kiểm tra chưa đạt: '+', '.join(failed or [result.get('error','runner failed')])+'.'))
        events=result.get('event_measurements',{})
        if events:lines.append('  Số đo sự kiện: `'+json.dumps(events,ensure_ascii=False)+'`.')
    lines+=['',f'Trong cặp đối chứng cùng cấu hình solver được chọn cho báo cáo, sai số pha cuối: '
               f'phản hồi **{comparison["feedback_final_formation"]:.5f} rad**, '
               f'vòng hở **{comparison["open_loop_final_formation"]:.5f} rad**.','']
    pair_file=sources['open_loop_failure']/'closest_physical_pair.json'
    if pair_file.exists():
        pair=json.loads(pair_file.read_text());shutil.copy(pair_file,target/'open_loop_closest_pair.json')
        lines+= [f'Trong ca vòng hở, hai xe vật lý **{pair["agents"][0]} và {pair["agents"][1]}** '
                 f'có khoảng cách tâm nhỏ nhất **{pair["min_physical_distance"]:.3f} m** '
                 f'ở khoảng **{pair["time"]:.1f} s**. Số contact là số mẫu tiếp xúc, không phải số sự kiện va chạm độc lập.','']
    if historical:
        lines+=['## Hồi quy sau sửa lỗi','',
                'Lượt năm xe ban đầu không phục hồi sau sự cố: MPC liên tục hết hạn khi khởi hành lại, '
                'khiến cả đội giữ lệnh dừng. Đã sửa giới hạn IPOPT theo wall time, khởi tạo dự đoán chuyển động '
                'bằng độ cong đường và tăng tốc khả thi, bỏ warm start cũ sau hold hoặc lỗi giải. '
                'Cũng sửa việc đặt lại pha sau gián đoạn odometry; reset thời gian thật vẫn đặt lại trạng thái.','',
                'Ma trận phía trên dùng các lượt chạy lại khi được cung cấp. Kết quả cũ được giữ lại, '
                'không ghi đè hoặc thay đổi ngưỡng kiểm thử:','']
        for name,old in historical.items():
            lines.append(f'- [{labels[name]} trước sửa](gazebo_suite/{name}_before_fix.json): '
                         f'{"ĐẠT" if old["passed"] else "CHƯA ĐẠT"}; '
                         f'lượt sau sửa lưu tại `{sources[name]}`.')
        lines+=['','Các lượt mới giữ nguyên mô hình, chi phí và giới hạn đầu vào MPCC; chỉ thay chính sách '
                 'khởi tạo và giới hạn thời gian giải. Metadata kèm theo ghi hash mã nguồn từng lượt.','']
    if args.faults:
        faults=json.loads((args.faults/'summary.json').read_text())
        lines+=['## Pause, reset và watchdog trên Gazebo','']
        for name,result in faults.items():
            (target/f'fault_{name}.json').write_text(json.dumps(result,indent=2)+'\n')
            lines.append(f'- **{name}:** '+('ĐẠT' if result['passed'] else 'CHƯA ĐẠT')+'. `'+json.dumps(result,ensure_ascii=False)+'`.')
    lines+=['','## Biểu đồ và dữ liệu','',
            'Vùng tô cam là khoảng dừng tác tử; đường đứt đỏ là thời điểm nhiễu pha. '
            'Các trường hợp vòng hở được dùng làm đối chứng, không phải chế độ khuyến nghị vận hành.','']
    for name in results:
        lines+= [f'### {labels.get(name,name)}','',f'![{name}](gazebo_suite/{name}.png)','',
                  f'[Kiểm tra và số đo](gazebo_suite/{name}.json) · [Cấu hình](gazebo_suite/{name}.yaml)','']
    relative=os.path.relpath(args.suite.resolve(),root/'validation')
    lines+= [f'Log và CSV đầy đủ: [{args.suite.name}]({relative}). Các thư mục `patrol_runs/` được gitignore.', '',
             'Không suy rộng kết quả sensing lý tưởng thành năng lực cảm biến thực. '
             'Bộ điều khiển vẫn không có ràng buộc tránh va chạm hoặc bảo đảm thời gian thực cứng; '
             'solver hết hạn gửi lệnh dừng. Kiểm thử headless không xác nhận giao diện GUI/RViz.','']
    (root/'validation/GAZEBO_SCENARIOS.md').write_text('\n'.join(lines))
    (target/'comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
    print(root/'validation/GAZEBO_SCENARIOS.md')


if __name__=='__main__':main()
