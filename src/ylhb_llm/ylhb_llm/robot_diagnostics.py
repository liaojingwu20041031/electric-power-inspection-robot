from __future__ import annotations

import time
import uuid
from typing import Any, Callable


class RobotDiagnosticEngine:
    SCOPES = {'all', 'connection', 'base', 'sensors', 'navigation', 'perception', 'patrol', 'voice'}

    def __init__(
        self,
        status_aggregator,
        network_status_provider,
        config: dict,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.status = status_aggregator
        self.network_status_provider = network_status_provider
        self.config = dict(config or {})
        self.clock = clock
        self.last_report: dict[str, Any] = {}
        self.diagnostic_freshness_sec = float(
            (self.config.get('freshness') or {}).get('system_status_sec') or 5.0)

    def get_connection_info(self, target: str = 'all') -> dict:
        system = self.status.get('system_status', self.clock())
        local_app = self.status.get('local_app_status', self.clock())
        cloud = self.status.get('cloud_status', self.clock())
        network = self.network_status_provider.snapshot()
        port = int(self.config.get('mobile_bridge_port') or 8000)
        endpoints = self.network_status_provider.app_endpoints('', port)
        warnings = list(network.get('warnings') or [])
        if system.get('mobile_bridge_core_state') != 'running':
            warnings.append({'code': 'BRIDGE_STOPPED', 'message': 'Mobile Bridge 未运行'})
        elif local_app.get('enabled') is False:
            warnings.append({'code': 'LOCAL_APP_DISABLED', 'message': '本地 APP 接口已关闭'})
        if not any(item.get('available') for item in endpoints):
            warnings.append({'code': 'NO_AVAILABLE_ENDPOINT', 'message': '未发现可用物理 IPv4 地址'})
        recommended = next((item for item in endpoints if item.get('available')), {})
        return {
            'bridge': {
                'owner': system.get('mobile_bridge_owner', 'unknown'),
                'core_state': system.get('mobile_bridge_core_state', 'unknown'),
                'tcp': system.get('mobile_bridge_tcp', 'unknown'),
                'local_app_enabled': local_app.get('enabled'),
                'cloud_enabled': cloud.get('enabled'),
            },
            'app_endpoints': endpoints if target != 'cloud' else [],
            'warnings': warnings,
            'recommended_endpoint': recommended if target != 'cloud' else {},
            'limitations': ['机器人只能确认本机接口和服务状态，不能直接确认手机当前连接了哪个网络'],
        }

    def run_self_check(self, scope: str = 'all') -> dict:
        if scope not in self.SCOPES:
            raise ValueError(f'unsupported diagnostic scope: {scope}')
        system = self.status.get('system_status', self.clock())
        mode = str(system.get('system_mode') or system.get('mode') or 'unknown').lower()
        checks: list[dict[str, Any]] = []
        issues: list[dict[str, Any]] = []
        dispatch = {
            'connection': self._diagnose_connection,
            'base': self._diagnose_base,
            'sensors': self._diagnose_sensors,
            'navigation': self._diagnose_navigation,
            'perception': self._diagnose_perception,
            'patrol': self._diagnose_patrol,
            'voice': self._diagnose_voice,
        }
        names = dispatch if scope == 'all' else {scope: dispatch[scope]}
        for name, diagnose in names.items():
            if scope == 'all' and name in {'base', 'sensors'} and system.get('bringup') not in {'running', 'embedded'}:
                checks.append(self._check(name, name, 'ok', '当前模式不要求底盘与传感器在线', [f'robot_mode={mode}']))
                continue
            if scope == 'all' and name == 'perception' and mode != 'inspection' and system.get('perception') != 'running':
                checks.append(self._check(name, name, 'ok', '当前模式不要求感知在线', [f'robot_mode={mode}']))
                continue
            if scope == 'all' and name == 'voice' and not self.status.raw('voice_status'):
                checks.append(self._check(name, name, 'ok', '当前未启用语音状态检查', ['voice status unavailable']))
                continue
            new_checks, new_issues = diagnose(system, mode)
            checks.extend(new_checks)
            issues.extend(new_issues)
        overall = 'failed' if any(item['severity'] == 'error' for item in issues) else ('warning' if issues else 'ok')
        report = {
            'schema_version': '1.0',
            'diagnostic_id': 'diag_' + uuid.uuid4().hex[:12],
            'scope': scope,
            'generated_at': self.clock(),
            'robot_mode': mode,
            'overall': overall,
            'checks': checks,
            'issues': issues,
            'connection': self.get_connection_info() if scope in {'all', 'connection'} else {},
            'recommended_next_step': issues[0]['user_actions'][0] if issues and issues[0].get('user_actions') else '当前检查范围未发现异常',
        }
        self.last_report = report
        return report

    @staticmethod
    def _check(identifier: str, component: str, status: str, summary: str, evidence: list[str]) -> dict:
        return {'id': identifier, 'component': component, 'status': status, 'summary': summary, 'evidence': evidence}

    @staticmethod
    def _issue(code: str, component: str, severity: str, summary: str, evidence: list[str], likely_causes: list[str], user_actions: list[str], recoverable=False, recovery_component=None) -> dict:
        return {
            'code': code, 'component': component, 'severity': severity, 'summary': summary,
            'evidence': evidence, 'likely_causes': likely_causes, 'confirmed_cause': False,
            'recoverable': bool(recoverable), 'recovery_component': recovery_component,
            'user_actions': user_actions,
        }

    def _diagnose_connection(self, system, _mode):
        info = self.get_connection_info()
        if info['bridge']['core_state'] == 'running' and info['bridge']['tcp'] == 'tcp_ok':
            return [self._check('bridge_core', 'mobile_bridge', 'ok', 'Mobile Bridge 正常', ['进程或外部服务正在运行', 'TCP 端口可达'])], []
        recoverable = info['bridge']['owner'] == 'supervisor'
        issue = self._issue('BRIDGE_STOPPED', 'mobile_bridge', 'error', 'Mobile Bridge 未运行', [f"core_state={info['bridge']['core_state']}", f"tcp={info['bridge']['tcp']}"], ['Bridge 进程未启动或已退出'], ['检查 Mobile Bridge 管理归属和最近错误'], recoverable, 'mobile_bridge' if recoverable else None)
        return [self._check('bridge_core', 'mobile_bridge', 'failed', issue['summary'], issue['evidence'])], [issue]

    def _diagnose_base(self, _system, _mode):
        chassis = self.status.get('chassis_status', self.clock())
        odom = self.status.get('odom', self.clock())
        if not chassis.get('fresh'):
            issue = self._issue('CHASSIS_STATUS_MISSING', 'base', 'error', '未收到新鲜底盘状态', ['底盘状态不存在或已过期'], ['bringup 或底盘驱动节点未运行'], ['检查底盘驱动节点和状态话题'])
            return [self._check('chassis', 'base', 'failed', issue['summary'], issue['evidence'])], [issue]
        if chassis.get('fault_latched') or chassis.get('state') == 'fault':
            fault = str(chassis.get('fault') or self.status.get('chassis_fault', self.clock()).get('text') or '底盘报告故障')
            issue = self._issue('CHASSIS_FAULT', 'base', 'error', fault, [fault], ['以驱动器真实故障码为准'], ['停止运动并人工检查驱动器故障'])
            return [self._check('chassis', 'base', 'failed', fault, [fault])], [issue]
        if chassis.get('heartbeat_seen') is False:
            issue = self._issue('CHASSIS_HEARTBEAT_MISSING', 'base', 'error', '底盘驱动未收到 CAN 心跳', ['heartbeat_seen=false', f"state={chassis.get('state')}"], ['驱动供电异常', 'CAN 线路或终端电阻异常', '驱动器 Node ID 不匹配'], ['确认电机驱动器电源指示灯', '检查 CAN-H、CAN-L、GND 和终端电阻', '检查驱动器 Node ID'])
            return [self._check('chassis', 'base', 'failed', issue['summary'], issue['evidence'])], [issue]
        if chassis.get('state') == 'online' and not odom.get('fresh'):
            issue = self._issue('ODOM_STALE', 'base', 'error', '底盘在线但里程计已停止更新', ['chassis state=online', 'odom stale'], ['里程计发布链路异常'], ['检查 /odom 发布者和时间戳'])
            return [self._check('chassis', 'base', 'failed', issue['summary'], issue['evidence'])], [issue]
        return [self._check('chassis', 'base', 'ok', '底盘通信正常', ['底盘状态新鲜', '/odom 新鲜'])], []

    def _diagnose_sensors(self, _system, _mode):
        checks, issues = [], []
        publishers = (_system.get('topic_publishers') or {}) if isinstance(_system, dict) else {}
        for source, label in (('scan', '雷达'), ('imu', 'IMU'), ('odom', '里程计')):
            item = self.status.get(source, self.clock())
            if item.get('fresh'):
                checks.append(self._check(source, 'sensors', 'ok', f'{label}数据正常', ['消息新鲜']))
            else:
                no_publisher = publishers.get(source) == 0
                code = f'{source.upper()}_NO_PUBLISHER' if no_publisher else f'{source.upper()}_STALE'
                evidence = ['发布者数量为 0'] if no_publisher else ['存在发布者，但消息不存在或已过期']
                likely = [f'{label}节点未运行'] if no_publisher else [f'{label}数据发布已停止']
                issue = self._issue(code, 'sensors', 'error', f'{label}无新鲜数据', evidence, likely, [f'检查 {source} 话题发布者和消息频率'])
                checks.append(self._check(source, 'sensors', 'failed', issue['summary'], issue['evidence']))
                issues.append(issue)
        return checks, issues

    def _diagnose_navigation(self, system, mode):
        required = mode in {'navigation', 'patrol', 'inspection'} or system.get('navigation') == 'running'
        if not required:
            return [self._check('navigation', 'navigation', 'ok', '当前模式不要求导航在线', [f'robot_mode={mode}'])], []
        readiness = system.get('patrol_readiness') or {}
        if readiness.get('nav2_active') is not True:
            issue = self._issue('NAV2_NOT_ACTIVE', 'navigation', 'error', 'Nav2 未处于 active 状态', ['nav2_active=false'], ['生命周期节点未激活', '导航进程启动失败'], ['检查 Supervisor startup_step、lifecycle 和 navigation_last_error'])
            return [self._check('nav2', 'navigation', 'failed', issue['summary'], issue['evidence'])], [issue]
        return [self._check('nav2', 'navigation', 'ok', '导航依赖正常', ['Nav2 active'])], []

    def _diagnose_perception(self, system, _mode):
        perception = system.get('perception')
        zed = system.get('zed')
        output = self.status.get('perception', self.clock())
        if perception != 'running':
            issue = self._issue('PERCEPTION_STOPPED', 'perception', 'error', '感知进程未运行', [f'perception={perception}'], ['感知进程退出或未启动'], ['查看 perception_last_error'], True, 'perception')
            return [self._check('perception', 'perception', 'failed', issue['summary'], issue['evidence'])], [issue]
        if zed != 'running':
            issue = self._issue('ZED_NOT_RUNNING', 'perception', 'warning', '感知进程运行，但相机输入尚未就绪', [f'zed={zed}'], ['ZED 进程未启动或相机不可用'], ['检查 ZED 进程、USB 和相机状态'])
            return [self._check('perception', 'perception', 'warning', issue['summary'], issue['evidence'])], [issue]
        if not output.get('fresh'):
            issue = self._issue('PERCEPTION_OUTPUT_STALE', 'perception', 'error', '感知进程和相机运行但没有新鲜输出', ['perception=running', 'zed=running', 'localized_objects stale'], ['感知节点内部异常或输入未送达'], ['检查感知进程最后错误和输出话题'])
            return [self._check('perception', 'perception', 'failed', issue['summary'], issue['evidence'])], [issue]
        return [self._check('perception', 'perception', 'ok', '感知输出正常', ['进程运行', '相机运行', '输出新鲜'])], []

    def _diagnose_patrol(self, system, mode):
        if mode not in {'patrol', 'inspection'}:
            return [self._check('patrol', 'patrol', 'ok', '当前没有活动巡逻', [f'robot_mode={mode}'])], []
        error = str(system.get('patrol_error') or '')
        if error:
            issue = self._issue('PATROL_ERROR', 'patrol', 'error', error, [error], ['以 Supervisor 巡逻启动步骤为准'], ['查看 startup_step 和 patrol_readiness'])
            return [self._check('patrol', 'patrol', 'failed', error, [error])], [issue]
        return [self._check('patrol', 'patrol', 'ok', '巡逻状态正常', ['无 patrol_error'])], []

    def _diagnose_voice(self, _system, _mode):
        voice = self.status.get('voice_status', self.clock())
        if voice.get('fresh'):
            return [self._check('voice', 'voice', 'ok', '语音状态可用', ['状态新鲜'])], []
        issue = self._issue('VOICE_STATUS_STALE', 'voice', 'warning', '语音状态不可用', ['语音状态不存在或已过期'], ['语音会话节点未运行'], ['检查语音会话状态话题'])
        return [self._check('voice', 'voice', 'warning', issue['summary'], issue['evidence'])], [issue]
