using System.Collections;
using UnityEngine;
using UnityEngine.EventSystems;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    public class ScenarioCardHoverLift : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
    {
        [SerializeField] private float hoverLift = 12f;
        [SerializeField] private float hoverScale = 1.03f;
        [SerializeField] private float transitionDuration = 0.16f;
        [SerializeField] private bool bringToFrontOnHover = true;

        private RectTransform _rectTransform;
        private Vector2 _baseAnchoredPosition;
        private Vector3 _baseScale;
        private int _baseSiblingIndex;
        private int _hoverRequestCount;
        private bool _isHovered;
        private Coroutine _tweenRoutine;

        private void Awake()
        {
            CacheTransformState();
        }

        private void OnEnable()
        {
            CacheTransformState();
        }

        private void OnDisable()
        {
            _hoverRequestCount = 0;
            _isHovered = false;
            StopTween();
            RestoreImmediate(restoreSibling: false);
        }

        public void Configure(float lift, float scale, float duration, bool bringToFront)
        {
            hoverLift = lift;
            hoverScale = Mathf.Max(1f, scale);
            transitionDuration = Mathf.Max(0.01f, duration);
            bringToFrontOnHover = bringToFront;
        }

        public void OnPointerEnter(PointerEventData eventData)
        {
            SetHoverRequest(true);
        }

        public void OnPointerExit(PointerEventData eventData)
        {
            SetHoverRequest(false);
        }

        public void SetHoverRequest(bool isHovering)
        {
            // LayoutGroup can settle after panel activation.
            // Refresh base just before the first hover request to prevent jump-to-origin on first interaction.
            if (isHovering && _hoverRequestCount == 0)
                CacheTransformState();

            _hoverRequestCount += isHovering ? 1 : -1;
            if (_hoverRequestCount < 0)
                _hoverRequestCount = 0;

            UpdateHoverState(_hoverRequestCount > 0);
        }

        public void RefreshBaseState()
        {
            CacheTransformState();
        }

        private void CacheTransformState()
        {
            if (_rectTransform == null)
                _rectTransform = GetComponent<RectTransform>();

            if (_rectTransform == null)
                return;

            _baseAnchoredPosition = _rectTransform.anchoredPosition;
            _baseScale = _rectTransform.localScale;
            _baseSiblingIndex = _rectTransform.GetSiblingIndex();
        }

        private void UpdateHoverState(bool shouldHover)
        {
            if (_rectTransform == null || _isHovered == shouldHover)
                return;

            _isHovered = shouldHover;

            if (shouldHover && bringToFrontOnHover)
            {
                _baseSiblingIndex = _rectTransform.GetSiblingIndex();
                TrySetAsLastSibling();
            }
            else if (!shouldHover && bringToFrontOnHover)
            {
                TryRestoreSiblingIndex();
            }

            StartTween();
        }

        private void StartTween()
        {
            StopTween();
            _tweenRoutine = StartCoroutine(TweenRoutine());
        }

        private void StopTween()
        {
            if (_tweenRoutine == null)
                return;

            StopCoroutine(_tweenRoutine);
            _tweenRoutine = null;
        }

        private IEnumerator TweenRoutine()
        {
            var startPos = _rectTransform.anchoredPosition;
            var startScale = _rectTransform.localScale;

            var targetPos = _isHovered
                ? _baseAnchoredPosition + new Vector2(0f, hoverLift)
                : _baseAnchoredPosition;
            var targetScale = _isHovered
                ? _baseScale * hoverScale
                : _baseScale;

            var elapsed = 0f;
            while (elapsed < transitionDuration)
            {
                elapsed += Time.unscaledDeltaTime;
                var t = Mathf.Clamp01(elapsed / transitionDuration);
                var eased = Mathf.SmoothStep(0f, 1f, t);

                _rectTransform.anchoredPosition = Vector2.LerpUnclamped(startPos, targetPos, eased);
                _rectTransform.localScale = Vector3.LerpUnclamped(startScale, targetScale, eased);
                yield return null;
            }

            _rectTransform.anchoredPosition = targetPos;
            _rectTransform.localScale = targetScale;
            _tweenRoutine = null;
        }

        private void RestoreImmediate(bool restoreSibling)
        {
            if (_rectTransform == null)
                return;

            _rectTransform.anchoredPosition = _baseAnchoredPosition;
            _rectTransform.localScale = _baseScale;

            if (restoreSibling && bringToFrontOnHover)
                TryRestoreSiblingIndex();
        }

        private void TrySetAsLastSibling()
        {
            if (_rectTransform == null || _rectTransform.parent == null)
                return;

            if (!_rectTransform.gameObject.activeSelf || !_rectTransform.parent.gameObject.activeSelf)
                return;

            try
            {
                _rectTransform.SetAsLastSibling();
            }
            catch (UnityException)
            {
                // Ignore temporary hierarchy lock while parent activation state is changing.
            }
        }

        private void TryRestoreSiblingIndex()
        {
            if (_rectTransform == null || _rectTransform.parent == null)
                return;

            if (!_rectTransform.gameObject.activeSelf || !_rectTransform.parent.gameObject.activeSelf)
                return;

            var clampedIndex = Mathf.Clamp(_baseSiblingIndex, 0, _rectTransform.parent.childCount - 1);

            try
            {
                _rectTransform.SetSiblingIndex(clampedIndex);
            }
            catch (UnityException)
            {
                // Ignore temporary hierarchy lock while parent activation state is changing.
            }
        }
    }

    [DisallowMultipleComponent]
    public class ScenarioCardHoverRelay : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
    {
        [SerializeField] private ScenarioCardHoverLift target;

        public void SetTarget(ScenarioCardHoverLift hoverTarget)
        {
            target = hoverTarget;
        }

        public void OnPointerEnter(PointerEventData eventData)
        {
            if (target != null)
                target.SetHoverRequest(true);
        }

        public void OnPointerExit(PointerEventData eventData)
        {
            if (target != null)
                target.SetHoverRequest(false);
        }

        private void OnDisable()
        {
            if (target != null)
                target.SetHoverRequest(false);
        }
    }
}
