using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace SimulationMonitor.Runtime
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(RectTransform))]
    public class BottomCardCarousel : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler, IPointerClickHandler
    {
        [Header("References")]
        [SerializeField] private RectTransform viewport;
        [SerializeField] private RectTransform content;

        [Header("Layout")]
        [SerializeField] private int visibleCardCount = 4;
        [SerializeField] private float spacing = 12f;
        [SerializeField] private int leftPadding = 12;
        [SerializeField] private int rightPadding = 12;
        [SerializeField] private int topPadding = 15;
        [SerializeField] private int bottomPadding = 0;
        [SerializeField] private float cardHeight = 250f;

        [Header("Interaction")]
        [SerializeField] private float snapDuration = 0.18f;
        [SerializeField] private float swipeVelocityThreshold = 900f;
        [SerializeField] private bool enableWheelPaging = true;
        [SerializeField] private bool enableEdgeClickPaging = true;
        [SerializeField] [Range(0f, 1f)] private float edgeClickRatio = 0.25f;

        private readonly List<RectTransform> cards = new List<RectTransform>();

        private HorizontalLayoutGroup layoutGroup;
        private RectTransform selfRect;

        private bool isDragging;
        private bool suppressClick;
        private float dragVelocityX;
        private Vector2 lastPointerPosition;
        private float lastPointerTime;

        private float targetContentX;
        private float smoothVelocityX;
        private bool animating;

        private int currentIndex;
        private float cachedCardWidth;

        private void Reset()
        {
            selfRect = GetComponent<RectTransform>();
            AutoWire();
        }

        private void Awake()
        {
            selfRect = GetComponent<RectTransform>();
            AutoWire();
            EnsureLayoutGroup();
            CacheCards();
            RebuildLayout(true);
        }

        private void OnEnable()
        {
            AutoWire();
            EnsureLayoutGroup();
            CacheCards();
            RebuildLayout(true);
        }

        private void OnRectTransformDimensionsChange()
        {
            if (!isActiveAndEnabled)
                return;

            RebuildLayout(false);
        }

        private void OnTransformChildrenChanged()
        {
            if (!isActiveAndEnabled)
                return;

            CacheCards();
            RebuildLayout(false);
        }

        private void Update()
        {
            if (!isActiveAndEnabled)
                return;

            if (!isDragging && enableWheelPaging)
            {
                var wheel = Input.mouseScrollDelta.y;
                if (Mathf.Abs(wheel) > 0.01f)
                    SnapToIndex(currentIndex + (wheel < 0f ? 1 : -1), true);
            }

            if (!isDragging && animating && content != null)
            {
                var currentX = content.anchoredPosition.x;
                var nextX = Mathf.SmoothDamp(currentX, targetContentX, ref smoothVelocityX, snapDuration, Mathf.Infinity, Time.unscaledDeltaTime);
                SetContentX(nextX, true);

                if (Mathf.Abs(nextX - targetContentX) <= 0.05f && Mathf.Abs(smoothVelocityX) <= 2f)
                {
                    SetContentX(targetContentX, true);
                    smoothVelocityX = 0f;
                    animating = false;
                }
            }
        }

        public void Configure(RectTransform viewportRect, RectTransform contentRect)
        {
            viewport = viewportRect;
            content = contentRect;
            AutoWire();
            EnsureLayoutGroup();
            CacheCards();
            RebuildLayout(true);
        }

        public void SnapToNext()
        {
            SnapToIndex(currentIndex + 1, true);
        }

        public void SnapToPrevious()
        {
            SnapToIndex(currentIndex - 1, true);
        }

        public void SnapToIndex(int index, bool animated)
        {
            currentIndex = Mathf.Clamp(index, 0, GetMaxIndex());
            targetContentX = -currentIndex * GetStepWidth();

            if (!animated)
            {
                animating = false;
                smoothVelocityX = 0f;
                SetContentX(targetContentX, true);
                return;
            }

            animating = true;
        }

        public void OnBeginDrag(PointerEventData eventData)
        {
            if (eventData.button != PointerEventData.InputButton.Left || content == null)
                return;

            isDragging = true;
            suppressClick = false;
            animating = false;
            smoothVelocityX = 0f;
            dragVelocityX = 0f;
            lastPointerPosition = eventData.position;
            lastPointerTime = Time.unscaledTime;
        }

        public void OnDrag(PointerEventData eventData)
        {
            if (!isDragging || content == null)
                return;

            var scaleFactor = GetCanvasScaleFactor();
            var deltaX = eventData.delta.x / scaleFactor;
            SetContentX(content.anchoredPosition.x + deltaX, true);

            var now = Time.unscaledTime;
            var dt = Mathf.Max(0.0001f, now - lastPointerTime);
            dragVelocityX = (eventData.position.x - lastPointerPosition.x) / dt / scaleFactor;
            lastPointerPosition = eventData.position;
            lastPointerTime = now;

            if (Mathf.Abs(eventData.position.x - eventData.pressPosition.x) > 4f)
                suppressClick = true;
        }

        public void OnEndDrag(PointerEventData eventData)
        {
            if (!isDragging || content == null)
                return;

            isDragging = false;

            var step = GetStepWidth();
            var rawIndex = -content.anchoredPosition.x / Mathf.Max(1f, step);
            var index = Mathf.RoundToInt(rawIndex);

            if (Mathf.Abs(dragVelocityX) > swipeVelocityThreshold)
                index += dragVelocityX < 0f ? 1 : -1;

            SnapToIndex(index, true);
        }

        public void OnPointerClick(PointerEventData eventData)
        {
            if (!enableEdgeClickPaging || suppressClick)
                return;

            if (eventData.button != PointerEventData.InputButton.Left || viewport == null)
                return;

            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(viewport, eventData.position, eventData.pressEventCamera, out var localPoint))
                return;

            var width = viewport.rect.width;
            if (width <= 0f)
                return;

            var normalizedX = (localPoint.x / width) * 2f;
            var threshold = Mathf.Clamp01(edgeClickRatio);

            if (normalizedX <= -threshold)
                SnapToPrevious();
            else if (normalizedX >= threshold)
                SnapToNext();
        }

        private void AutoWire()
        {
            if (selfRect == null)
                selfRect = GetComponent<RectTransform>();

            if (viewport == null)
                viewport = selfRect;

            if (content == null)
            {
                var existing = transform.Find("ScrollContent") as RectTransform;
                if (existing != null)
                    content = existing;
            }
        }

        private void EnsureLayoutGroup()
        {
            if (content == null)
                return;

            layoutGroup = content.GetComponent<HorizontalLayoutGroup>();
            if (layoutGroup == null)
                layoutGroup = content.gameObject.AddComponent<HorizontalLayoutGroup>();

            layoutGroup.spacing = spacing;
            layoutGroup.padding = new RectOffset(leftPadding, rightPadding, topPadding, bottomPadding);
            layoutGroup.childAlignment = TextAnchor.UpperLeft;
            layoutGroup.childControlWidth = true;
            layoutGroup.childControlHeight = true;
            layoutGroup.childForceExpandWidth = false;
            layoutGroup.childForceExpandHeight = false;
        }

        private void CacheCards()
        {
            cards.Clear();

            if (content == null)
                return;

            for (var i = 0; i < content.childCount; i++)
            {
                var child = content.GetChild(i) as RectTransform;
                if (child == null || !child.gameObject.activeSelf)
                    continue;

                DisableCardRaycastTargets(child);
                cards.Add(child);
            }
        }

        private static void DisableCardRaycastTargets(RectTransform card)
        {
            if (card == null)
                return;

            var graphics = card.GetComponentsInChildren<Graphic>(true);
            for (var i = 0; i < graphics.Length; i++)
            {
                if (graphics[i] != null)
                    graphics[i].raycastTarget = false;
            }
        }

        private void RebuildLayout(bool immediate)
        {
            if (viewport == null || content == null)
                return;

            EnsureLayoutGroup();

            var viewportWidth = viewport.rect.width;
            var viewportHeight = viewport.rect.height;
            if (viewportWidth <= 0f || viewportHeight <= 0f)
                return;

            var targetVisibleCount = Mathf.Max(1, visibleCardCount);
            var availableWidth = viewportWidth
                                 - layoutGroup.padding.left
                                 - layoutGroup.padding.right
                                 - layoutGroup.spacing * (targetVisibleCount - 1);

            cachedCardWidth = Mathf.Max(1f, availableWidth / targetVisibleCount);
            var targetCardHeight = Mathf.Max(1f, cardHeight);

            for (var i = 0; i < cards.Count; i++)
            {
                var card = cards[i];
                var element = card.GetComponent<LayoutElement>();
                if (element == null)
                    element = card.gameObject.AddComponent<LayoutElement>();

                element.minWidth = cachedCardWidth;
                element.preferredWidth = cachedCardWidth;
                element.flexibleWidth = 0f;
                element.minHeight = targetCardHeight;
                element.preferredHeight = targetCardHeight;
                element.flexibleHeight = 0f;
            }

            LayoutRebuilder.ForceRebuildLayoutImmediate(content);

            var contentWidth = (float)(layoutGroup.padding.left + layoutGroup.padding.right);
            if (cards.Count > 0)
                contentWidth += cards.Count * cachedCardWidth + Mathf.Max(0, cards.Count - 1) * layoutGroup.spacing;

            var contentHeight = Mathf.Max(viewportHeight, layoutGroup.padding.top + layoutGroup.padding.bottom + targetCardHeight);
            content.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, contentWidth);
            content.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, contentHeight);

            currentIndex = Mathf.Clamp(currentIndex, 0, GetMaxIndex());
            targetContentX = -currentIndex * GetStepWidth();

            if (immediate)
            {
                animating = false;
                smoothVelocityX = 0f;
                SetContentX(targetContentX, true);
            }
            else if (!isDragging)
            {
                animating = true;
            }

            if (cards.Count <= targetVisibleCount)
            {
                SetContentX(0f, true);
                animating = false;
            }
        }

        private void SetContentX(float x, bool clamp)
        {
            if (content == null)
                return;

            var nextX = clamp ? Mathf.Clamp(x, GetMinContentX(), 0f) : x;
            var pos = content.anchoredPosition;
            pos.x = nextX;
            content.anchoredPosition = pos;
        }

        private int GetMaxIndex()
        {
            return Mathf.Max(0, cards.Count - Mathf.Max(1, visibleCardCount));
        }

        private float GetStepWidth()
        {
            return cachedCardWidth + (layoutGroup != null ? layoutGroup.spacing : spacing);
        }

        private float GetMinContentX()
        {
            return -GetMaxIndex() * GetStepWidth();
        }

        private float GetCanvasScaleFactor()
        {
            if (viewport == null)
                return 1f;

            var canvas = viewport.GetComponentInParent<Canvas>();
            if (canvas == null)
                return 1f;

            return Mathf.Max(0.0001f, canvas.scaleFactor);
        }
    }
}
