using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace GUI.Runtime
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Button))]
    public class UIButtonSfxEmitter : MonoBehaviour
    {
        [SerializeField] private UIButtonSfxRouter router;

        private Button _button;

        private void Awake()
        {
            _button = GetComponent<Button>();
            if (router == null)
                router = GetComponentInParent<UIButtonSfxRouter>(true);
        }

        private void OnEnable()
        {
            if (_button == null)
                _button = GetComponent<Button>();

            _button.onClick.AddListener(OnButtonClick);
        }

        private void OnDisable()
        {
            if (_button != null)
                _button.onClick.RemoveListener(OnButtonClick);
        }

        public void SetRouter(UIButtonSfxRouter targetRouter)
        {
            router = targetRouter;
        }

        private void OnButtonClick()
        {
            if (EventSystem.current != null && EventSystem.current.currentSelectedGameObject == gameObject)
                EventSystem.current.SetSelectedGameObject(null);

            router?.PlayClick();
        }
    }
}
