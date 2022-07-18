using UnityEngine;
using UnityEditor;
using UnityEngine.UI;
using System.Collections;
using BodyPartSegmentation.UI;      

namespace BodyPartSegmentation.Editor.UI{
    [CustomEditor(typeof(StatusImage))]
    public class StatusImageEditor : UnityEditor.UI.ImageEditor
    {
        protected StatusImage component;

        public override void OnInspectorGUI()
        {
            component = (StatusImage)target;

            base.OnInspectorGUI();

            component.FalseColor = EditorGUILayout.ColorField("False Color", component.FalseColor);
            component.TrueColor = EditorGUILayout.ColorField("True Color", component.TrueColor);
            ExpandUI();
            if (GUI.changed)
            {
                component.color = component.FalseColor;
            }
        }

        protected virtual void ExpandUI(){}
    }
}