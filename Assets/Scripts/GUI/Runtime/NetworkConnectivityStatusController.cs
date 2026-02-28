using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    public class NetworkConnectivityStatusController : MonoBehaviour
    {
        private enum ConnectivityState
        {
            Disconnected,
            Connecting,
            Connected
        }

        [Header("Targets")]
        [SerializeField] private TMP_Text statusText;
        [SerializeField] private Graphic statusDot;
        [SerializeField] private TMP_Text latencyText;

        [Header("Connectivity Check")]
        [SerializeField] private string checkUrl = "https://www.gstatic.com/generate_204";
        [SerializeField, Min(2f)] private float checkIntervalSeconds = 8f;
        [SerializeField, Min(1)] private int timeoutSeconds = 4;

        [Header("Palette")]
        [SerializeField] private Color disconnectedColor = new Color32(217, 74, 74, 255);
        [SerializeField] private Color connectingColor = new Color32(232, 174, 58, 255);
        [SerializeField] private Color connectedColor = new Color32(68, 181, 106, 255);
        [SerializeField] private Color statusTextColor = Color.black;

        private Coroutine _pollingRoutine;
        private WaitForSecondsRealtime _waitInterval;

        private void Awake()
        {
            ResolveReferences();
            _waitInterval = new WaitForSecondsRealtime(checkIntervalSeconds);
        }

        private void Reset()
        {
            ResolveReferences();
        }

        private void OnEnable()
        {
            if (_waitInterval == null)
                _waitInterval = new WaitForSecondsRealtime(checkIntervalSeconds);

            StartPolling();
        }

        private void OnDisable()
        {
            if (_pollingRoutine != null)
            {
                StopCoroutine(_pollingRoutine);
                _pollingRoutine = null;
            }
        }

        private void OnValidate()
        {
            checkIntervalSeconds = Mathf.Max(2f, checkIntervalSeconds);
            timeoutSeconds = Mathf.Max(1, timeoutSeconds);
            _waitInterval = new WaitForSecondsRealtime(checkIntervalSeconds);

            if (!Application.isPlaying)
                ResolveReferences();
        }

        private void StartPolling()
        {
            if (_pollingRoutine != null)
                StopCoroutine(_pollingRoutine);

            _pollingRoutine = StartCoroutine(PollConnectivityLoop());
        }

        private IEnumerator PollConnectivityLoop()
        {
            while (enabled && gameObject.activeInHierarchy)
            {
                yield return CheckConnectivityOnce();
                yield return _waitInterval;
            }
        }

        private IEnumerator CheckConnectivityOnce()
        {
            if (Application.internetReachability == NetworkReachability.NotReachable)
            {
                ApplyState(ConnectivityState.Disconnected);
                SetLatencyUnavailable();
                yield break;
            }

            ApplyState(ConnectivityState.Connecting);
            SetLatencyPending();

            using (var request = UnityWebRequest.Get(checkUrl))
            {
                var startedAt = Time.realtimeSinceStartup;
                request.timeout = timeoutSeconds;
                request.SetRequestHeader("Cache-Control", "no-cache");
                request.SetRequestHeader("Pragma", "no-cache");

                yield return request.SendWebRequest();

                var succeeded = request.result == UnityWebRequest.Result.Success;
                var responseCodeOk = request.responseCode == 204 ||
                                     (request.responseCode >= 200 && request.responseCode < 400);
                var isConnected = succeeded && responseCodeOk;

                ApplyState(isConnected ? ConnectivityState.Connected : ConnectivityState.Disconnected);

                if (isConnected)
                {
                    var elapsedSeconds = Time.realtimeSinceStartup - startedAt;
                    SetLatencyMilliseconds(elapsedSeconds * 1000f);
                }
                else
                {
                    SetLatencyUnavailable();
                }
            }
        }

        private void ApplyState(ConnectivityState state)
        {
            Color stateColor;
            string stateLabel;

            switch (state)
            {
                case ConnectivityState.Connected:
                    stateColor = connectedColor;
                    stateLabel = "System Status:Online";
                    break;
                case ConnectivityState.Connecting:
                    stateColor = connectingColor;
                    stateLabel = "System Status:Connecting";
                    break;
                default:
                    stateColor = disconnectedColor;
                    stateLabel = "System Status:Connection Failed";
                    break;
            }

            if (statusDot != null)
                statusDot.color = stateColor;

            if (statusText != null)
            {
                statusText.text = stateLabel;
                statusText.color = statusTextColor;
            }
        }

        private void SetLatencyPending()
        {
            if (latencyText != null)
                latencyText.text = "Latency: ...";
        }

        private void SetLatencyUnavailable()
        {
            if (latencyText != null)
                latencyText.text = "Latency: --ms";
        }

        private void SetLatencyMilliseconds(float milliseconds)
        {
            if (latencyText == null)
                return;

            var clamped = Mathf.Max(0f, milliseconds);
            var rounded = Mathf.RoundToInt(clamped);
            latencyText.text = $"Latency: {rounded}ms";
        }

        private void ResolveReferences()
        {
            if (statusText == null)
            {
                var textTransform = transform.Find("Text");
                statusText = textTransform != null
                    ? textTransform.GetComponent<TMP_Text>()
                    : GetComponentInChildren<TMP_Text>(true);
            }

            if (statusDot == null)
            {
                var dotTransform = transform.Find("StatusDot");
                if (dotTransform != null)
                    statusDot = dotTransform.GetComponent<Graphic>();
            }

            if (latencyText == null)
            {
                var latencyTransform = transform.parent != null
                    ? transform.parent.Find("Latency/Text")
                    : null;

                if (latencyTransform != null)
                    latencyText = latencyTransform.GetComponent<TMP_Text>();
            }
        }
    }
}
