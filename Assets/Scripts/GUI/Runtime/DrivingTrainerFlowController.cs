using System.Collections;
using UnityEngine;
using UnityEngine.UI;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    public class DrivingTrainerFlowController : MonoBehaviour
    {
        [Header("Panels")]
        [SerializeField] private RectTransform drivingTrainerUI;
        [SerializeField] private RectTransform scenarioSelectionUI;

        [Header("Entry")]
        [SerializeField] private Button startSimulationButton;
        [SerializeField] private bool forceDrivingPanelOnStart = true;

        [Header("Transition")]
        [SerializeField] private float transitionDuration = 0.35f;
        [SerializeField] private float slideDistance = 56f;
        [SerializeField] private bool useUnscaledTime = true;

        [Header("Card Hover")]
        [SerializeField] private float cardHoverLift = 12f;
        [SerializeField] private float cardHoverScale = 1.03f;
        [SerializeField] private float cardHoverDuration = 0.16f;
        [SerializeField] private bool bringCardToFrontOnHover = false;

        private CanvasGroup _drivingGroup;
        private CanvasGroup _scenarioGroup;
        private RectTransform _activePanel;
        private Coroutine _transitionRoutine;

        private void Reset()
        {
            AutoWire();
        }

        private void Awake()
        {
            AutoWire();
            EnsureScenarioPaginationController();
            EnsureCanvasGroups();
            SetupScenarioCardHoverEffects();
            ApplyInitialPanelState();
        }

        private void OnEnable()
        {
            if (startSimulationButton != null)
                startSimulationButton.onClick.AddListener(HandleStartSimulationClicked);
        }

        private void OnDisable()
        {
            if (startSimulationButton != null)
                startSimulationButton.onClick.RemoveListener(HandleStartSimulationClicked);
        }

        [ContextMenu("Show Scenario Selection")]
        public void ShowScenarioSelection()
        {
            if (drivingTrainerUI == null || scenarioSelectionUI == null)
                return;

            if (_activePanel == scenarioSelectionUI)
                return;

            StartPanelTransition(
                drivingTrainerUI,
                _drivingGroup,
                scenarioSelectionUI,
                _scenarioGroup,
                forward: true);
        }

        [ContextMenu("Show Driving Trainer")]
        public void ShowDrivingTrainer()
        {
            if (drivingTrainerUI == null || scenarioSelectionUI == null)
                return;

            if (_activePanel == drivingTrainerUI)
                return;

            StartPanelTransition(
                scenarioSelectionUI,
                _scenarioGroup,
                drivingTrainerUI,
                _drivingGroup,
                forward: false);
        }

        private void HandleStartSimulationClicked()
        {
            ShowScenarioSelection();
        }

        private void AutoWire()
        {
            if (drivingTrainerUI == null)
                drivingTrainerUI = transform.Find("DrivingTrainerUI") as RectTransform;

            if (scenarioSelectionUI == null)
                scenarioSelectionUI = transform.Find("ScenarioSelectionUI") as RectTransform;

            if (startSimulationButton == null && drivingTrainerUI != null)
            {
                var startButtonTransform = drivingTrainerUI.Find("Center/StartButton");
                if (startButtonTransform != null)
                    startSimulationButton = startButtonTransform.GetComponent<Button>();
            }
        }

        private void EnsureScenarioPaginationController()
        {
            if (scenarioSelectionUI == null)
                return;

            var pagination = scenarioSelectionUI.GetComponent<ScenarioCardPaginationController>();
            if (pagination == null)
                pagination = scenarioSelectionUI.gameObject.AddComponent<ScenarioCardPaginationController>();

            if (!pagination.enabled)
                pagination.enabled = true;
        }

        private void EnsureCanvasGroups()
        {
            _drivingGroup = EnsureCanvasGroup(drivingTrainerUI);
            _scenarioGroup = EnsureCanvasGroup(scenarioSelectionUI);
        }

        private static CanvasGroup EnsureCanvasGroup(RectTransform panel)
        {
            if (panel == null)
                return null;

            var group = panel.GetComponent<CanvasGroup>();
            if (group == null)
                group = panel.gameObject.AddComponent<CanvasGroup>();

            return group;
        }

        private void ApplyInitialPanelState()
        {
            if (drivingTrainerUI == null || scenarioSelectionUI == null)
                return;

            if (forceDrivingPanelOnStart)
            {
                SetPanelVisibility(drivingTrainerUI, _drivingGroup, true);
                SetPanelVisibility(scenarioSelectionUI, _scenarioGroup, false);
                _activePanel = drivingTrainerUI;
                return;
            }

            var drivingActive = drivingTrainerUI.gameObject.activeSelf;
            var scenarioActive = scenarioSelectionUI.gameObject.activeSelf;
            _activePanel = scenarioActive && !drivingActive ? scenarioSelectionUI : drivingTrainerUI;

            SetPanelVisibility(drivingTrainerUI, _drivingGroup, _activePanel == drivingTrainerUI);
            SetPanelVisibility(scenarioSelectionUI, _scenarioGroup, _activePanel == scenarioSelectionUI);
        }

        private static void SetPanelVisibility(RectTransform panel, CanvasGroup group, bool visible)
        {
            if (panel == null || group == null)
                return;

            panel.gameObject.SetActive(true);
            panel.anchoredPosition = Vector2.zero;
            group.alpha = visible ? 1f : 0f;
            group.interactable = visible;
            group.blocksRaycasts = visible;

            if (!visible)
                panel.gameObject.SetActive(false);
        }

        private void StartPanelTransition(
            RectTransform fromPanel,
            CanvasGroup fromGroup,
            RectTransform toPanel,
            CanvasGroup toGroup,
            bool forward)
        {
            if (fromPanel == null || fromGroup == null || toPanel == null || toGroup == null)
            {
                SetPanelVisibility(fromPanel, fromGroup, false);
                SetPanelVisibility(toPanel, toGroup, true);
                _activePanel = toPanel;
                return;
            }

            if (_transitionRoutine != null)
                StopCoroutine(_transitionRoutine);

            _transitionRoutine = StartCoroutine(TransitionRoutine(
                fromPanel,
                fromGroup,
                toPanel,
                toGroup,
                forward));
        }

        private IEnumerator TransitionRoutine(
            RectTransform fromPanel,
            CanvasGroup fromGroup,
            RectTransform toPanel,
            CanvasGroup toGroup,
            bool forward)
        {
            toPanel.gameObject.SetActive(true);

            var fromStartPos = Vector2.zero;
            var fromEndPos = new Vector2(forward ? -slideDistance : slideDistance, 0f);
            var toStartPos = new Vector2(forward ? slideDistance : -slideDistance, 0f);
            var toEndPos = Vector2.zero;

            fromPanel.anchoredPosition = fromStartPos;
            toPanel.anchoredPosition = toStartPos;

            fromGroup.alpha = 1f;
            fromGroup.interactable = false;
            fromGroup.blocksRaycasts = false;

            toGroup.alpha = 0f;
            toGroup.interactable = false;
            toGroup.blocksRaycasts = false;

            var elapsed = 0f;
            var duration = Mathf.Max(0.01f, transitionDuration);

            while (elapsed < duration)
            {
                elapsed += useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
                var t = Mathf.Clamp01(elapsed / duration);
                var eased = Mathf.SmoothStep(0f, 1f, t);

                fromPanel.anchoredPosition = Vector2.LerpUnclamped(fromStartPos, fromEndPos, eased);
                toPanel.anchoredPosition = Vector2.LerpUnclamped(toStartPos, toEndPos, eased);

                fromGroup.alpha = Mathf.LerpUnclamped(1f, 0f, eased);
                toGroup.alpha = Mathf.LerpUnclamped(0f, 1f, eased);
                yield return null;
            }

            fromPanel.anchoredPosition = Vector2.zero;
            fromGroup.alpha = 0f;
            fromPanel.gameObject.SetActive(false);

            toPanel.anchoredPosition = Vector2.zero;
            toGroup.alpha = 1f;
            toGroup.interactable = true;
            toGroup.blocksRaycasts = true;

            if (toPanel == scenarioSelectionUI)
                StartCoroutine(RefreshScenarioCardHoverBaseNextFrame());

            _activePanel = toPanel;
            _transitionRoutine = null;
        }

        private IEnumerator RefreshScenarioCardHoverBaseNextFrame()
        {
            // Wait one frame so layout groups finish their first pass after panel activation.
            yield return null;

            if (scenarioSelectionUI == null || !scenarioSelectionUI.gameObject.activeInHierarchy)
                yield break;

            var cardRow = scenarioSelectionUI.Find("MainSection/CardsArea/CardRow") as RectTransform;
            if (cardRow == null)
                yield break;

            LayoutRebuilder.ForceRebuildLayoutImmediate(cardRow);

            for (var i = 0; i < cardRow.childCount; i++)
            {
                var card = cardRow.GetChild(i);
                if (card == null || !card.gameObject.activeInHierarchy)
                    continue;

                var hover = card.GetComponent<ScenarioCardHoverLift>();
                if (hover != null)
                    hover.RefreshBaseState();
            }
        }

        private void SetupScenarioCardHoverEffects()
        {
            if (scenarioSelectionUI == null)
                return;

            var cardRow = scenarioSelectionUI.Find("MainSection/CardsArea/CardRow");
            if (cardRow == null)
                return;

            // Reordering siblings during hover causes card-row relayout jitter.
            // Keep stable order when cards are inside a LayoutGroup container.
            var hasLayoutGroupParent = cardRow.GetComponent<LayoutGroup>() != null;
            var allowBringToFront = bringCardToFrontOnHover && !hasLayoutGroupParent;

            for (var i = 0; i < cardRow.childCount; i++)
            {
                var card = cardRow.GetChild(i);
                if (card == null || !card.name.StartsWith("Card_", System.StringComparison.Ordinal))
                    continue;

                var hover = card.GetComponent<ScenarioCardHoverLift>();
                if (hover == null)
                    hover = card.gameObject.AddComponent<ScenarioCardHoverLift>();

                hover.Configure(cardHoverLift, cardHoverScale, cardHoverDuration, allowBringToFront);
                ConfigureCardRaycastTargets(card);
                AttachHoverRelays(card, hover);
            }
        }

        private static void AttachHoverRelays(Transform card, ScenarioCardHoverLift hover)
        {
            var buttons = card.GetComponentsInChildren<Button>(true);
            foreach (var button in buttons)
            {
                if (button == null)
                    continue;

                var relay = button.GetComponent<ScenarioCardHoverRelay>();
                if (relay == null)
                    relay = button.gameObject.AddComponent<ScenarioCardHoverRelay>();

                relay.SetTarget(hover);
            }
        }

        private static void ConfigureCardRaycastTargets(Transform card)
        {
            var rootGraphic = card.GetComponent<Graphic>();
            if (rootGraphic == null)
            {
                var fallbackImage = card.GetComponent<Image>();
                if (fallbackImage == null)
                    fallbackImage = card.gameObject.AddComponent<Image>();

                // Transparent receiver so cards without a root graphic still get hover events.
                fallbackImage.color = new Color(1f, 1f, 1f, 0.001f);
                fallbackImage.raycastTarget = true;
                rootGraphic = fallbackImage;
            }
            else
            {
                rootGraphic.raycastTarget = true;
            }

            var selectables = card.GetComponentsInChildren<Selectable>(true);
            var graphics = card.GetComponentsInChildren<Graphic>(true);

            foreach (var graphic in graphics)
            {
                if (graphic == null || graphic.transform == card)
                    continue;

                if (IsUnderSelectable(graphic.transform, selectables))
                    continue;

                graphic.raycastTarget = false;
            }
        }

        private static bool IsUnderSelectable(Transform target, Selectable[] selectables)
        {
            foreach (var selectable in selectables)
            {
                if (selectable != null && target.IsChildOf(selectable.transform))
                    return true;
            }

            return false;
        }
    }
}
