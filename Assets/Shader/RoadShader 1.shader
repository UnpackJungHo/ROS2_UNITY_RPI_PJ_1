Shader "Custom/RealisticRoadShader"
{
    Properties
    {
        // [변경] 색상 대신 텍스처를 입력받습니다.
        _RoadTexture ("Road Texture (Asphalt)", 2D) = "white" {}
        _LineTexture ("Line Texture (Paint)", 2D) = "white" {}
        
        // 기존 속성들
        _CenterWidth ("Center Line Width", Range(0, 0.5)) = 0.02
        _EdgeWidth ("Edge Line Width", Range(0, 0.5)) = 0.05 
        _DashFrequency ("Dash Frequency", Float) = 4
        _DashRatio ("Dash Ratio", Range(0, 1)) = 0.5
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            // 텍스처 타일링을 위해 필요
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                // 텍스처마다 다른 타일링을 위해 별도의 UV 좌표가 필요할 수 있음
                float2 uv_Road : TEXCOORD1;
            };

            // [변경] 텍스처 변수 선언
            sampler2D _RoadTexture;
            float4 _RoadTexture_ST; // Tiling/Offset을 위한 변수

            sampler2D _LineTexture;
            // 페인트는 보통 반복되지 않고 늘어나므로 _ST는 생략 가능하지만, 필요시 추가

            // 기존 변수들
            float _CenterWidth;
            float _EdgeWidth;
            float _DashFrequency;
            float _DashRatio;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                // 기본 UV
                o.uv = v.uv;
                // 도로 텍스처에 대한 Tiling/Offset 적용
                o.uv_Road = TRANSFORM_TEX(v.uv, _RoadTexture);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float2 uv = i.uv;

                // --- 마스크 계산 (기존과 동일) ---
                // 가장자리 선
                float leftEdge = step(uv.x, _EdgeWidth);
                float rightEdge = step(1.0 - _EdgeWidth, uv.x);
                float edgeMask = leftEdge + rightEdge;

                // 중앙 점선
                float centerLinePos = abs(uv.x - 0.5);
                float centerLineMask = step(centerLinePos, _CenterWidth * 0.5);
                float dashPattern = step(frac(uv.y * _DashFrequency + (_DashRatio * 0.5)), _DashRatio);
                float finalCenterLine = centerLineMask * dashPattern;

                // 최종 마스크 (1이면 선, 0이면 도로)
                float finalMask = saturate(edgeMask + finalCenterLine);

                // --- [핵심 변경] 텍스처 샘플링 및 색상 혼합 ---
                
                // 1. 아스팔트 텍스처에서 색상 가져오기 (Tiling 적용된 UV 사용)
                fixed4 roadColor = tex2D(_RoadTexture, i.uv_Road);
                
                // 2. 페인트 텍스처에서 색상 가져오기
                // 페인트 질감은 도로 전체 UV에 매핑되거나, 마스크 영역에만 매핑될 수 있음.
                // 여기서는 도로 전체 UV를 사용하여 자연스럽게 늘어나거나 반복되게 함.
                fixed4 lineColor = tex2D(_LineTexture, uv);

                // 3. 마스크를 사용하여 두 텍스처를 혼합 (lerp)
                // finalMask가 0인 곳은 roadColor, 1인 곳은 lineColor가 됨
                fixed4 finalColor = lerp(roadColor, lineColor, finalMask);
                
                return finalColor;
            }
            ENDCG
        }
    }
}