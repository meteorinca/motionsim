const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../main/index.html'), 'utf8');
const elements = new Map();
const element = id => {
  if (!elements.has(id)) elements.set(id, {
    value: '', textContent: '', style: {}, classList: { add() { }, remove() { } },
    getContext() { return {}; }
  });
  return elements.get(id);
};
let stats = {
  m1: { target: 90, actual: 90, error: 0, duty: 0 },
  pid: { kp: 14, ki: .2, kd: 1.8 }, pid_revision: 0,
  as5600: { raw: 1024, magnet_ok: true }, autotune: { running: false, status: 'Idle' },
  version: 'v0.4', pkt_rate_hz: 0, rssi: -50, ip: '127.0.0.1'
};
const context = vm.createContext({
  document: { getElementById: element }, window: { addEventListener() { } },
  setInterval() { }, setTimeout() { }, clearInterval() { }, console,
  fetch: async () => ({ ok: true, json: async () => stats })
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
(async () => {
  await context.poll();
  assert.equal(element('kp').value, 14);
  assert.equal(element('fw-badge').textContent, 'v0.4');
  element('kp').value = '22';
  await context.poll();
  assert.equal(element('kp').value, '22', 'poll must preserve local edits without a controller change');
  stats = { ...stats, pid: { kp: 14.7, ki: .22, kd: 1.8 }, pid_revision: 1 };
  await context.poll();
  assert.equal(element('kp').value, 14.7);
  assert.equal(element('ki').value, .22);
  stats = { ...stats, pid: { kp: 14, ki: .2, kd: 1.8 }, pid_revision: 2 };
  await context.poll();
  assert.equal(element('kp').value, 14, 'rollback must refresh displayed gains');
  context.fetch = async () => ({ ok: false, text: async () => 'Calibration running' });
  await context.setPid();
  assert.equal(element('tune-msg').textContent, 'Calibration running');
  const goodSensor = {raw: 4095, status_valid: true, magnet_ok: true, sample_age_ms: 0, agc: 64};
  const wrap = context.summarizeEncoderSamples([goodSensor, {...goodSensor, raw: 0}, goodSensor]);
  assert.equal(wrap.peakToPeakDeg, 360 / 4096, 'raw rollover must not look like a full turn of jitter');
  const fault = context.summarizeEncoderSamples([goodSensor, {...goodSensor, status_valid: false}]);
  assert.equal(fault.unhealthySamples, 1);
  context.fetch = async url => {
    assert.equal(url, '/api/stats', 'stationary check must never command a motor');
    return {ok: true, json: async () => ({...stats, armed: true})};
  };
  await context.checkEncoder();
  assert.match(element('encoder-check-result').textContent, /SAFE DISARM/);
  context.setTimeout = callback => callback();
  context.fetch = async url => {
    assert.equal(url, '/api/stats');
    return {ok: true, json: async () => ({...stats, armed: false, as5600: goodSensor})};
  };
  await context.checkEncoder();
  assert.match(element('encoder-check-result').textContent, /No gross stationary jitter/);
  assert.equal(element('encoder-check-btn').disabled, false);
  console.log('PASS stationary encoder check, wraparound, fault status and read-only motor gate');
  console.log('PASS Web UI version, gain refresh, edit preservation, rollback and API errors');
})().catch(error => { console.error(error); process.exitCode = 1; });
