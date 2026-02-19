using UnityEngine;

/// <summary>
/// ROS 토픽 네임스페이스 관리자.
/// 차량 오브젝트(또는 부모)에 하나 부착하면,
/// 하위 모든 ROS Publisher/Subscriber가 이 prefix를 참조하여 토픽 이름을 결정합니다.
///
/// 예: namespace = "/amr0" → "/camera/image_raw" → "/amr0/camera/image_raw"
///     namespace = ""      → "/camera/image_raw" → "/camera/image_raw" (기본, 변경 없음)
/// </summary>
public class RosTopicNamespace : MonoBehaviour
{
    [Header("Namespace")]
    [Tooltip("토픽 앞에 붙을 prefix (예: /amr0). 비어있으면 기본 토픽 이름 사용")]
    public string namespacePrefix = "";

    /// <summary>
    /// 토픽 이름에 namespace prefix를 적용합니다.
    /// </summary>
    public string Apply(string topicName)
    {
        if (string.IsNullOrEmpty(namespacePrefix))
            return topicName;

        // prefix가 /로 끝나면 제거 (중복 방지)
        string prefix = namespacePrefix.TrimEnd('/');

        // topicName이 /로 시작하면 그대로, 아니면 / 추가
        if (!topicName.StartsWith("/"))
            topicName = "/" + topicName;

        return prefix + topicName;
    }

    /// <summary>
    /// 계층 구조에서 RosTopicNamespace를 찾아 토픽 이름을 변환합니다.
    /// 컴포넌트가 없으면 원본 토픽 이름을 그대로 반환합니다.
    /// </summary>
    public static string Resolve(GameObject obj, string topicName)
    {
        if (obj == null)
            return topicName;

        var ns = obj.GetComponentInParent<RosTopicNamespace>();
        if (ns == null)
            return topicName;

        return ns.Apply(topicName);
    }
}
