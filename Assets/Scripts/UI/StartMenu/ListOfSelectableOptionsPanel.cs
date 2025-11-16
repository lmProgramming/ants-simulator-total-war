using System.Collections.Generic;
using UnityEngine;

public class ListOfSelectableOptionsPanel : MonoBehaviour
{
    public List<SelectableOptionsStruct> selectableStructs;

    public List<SelectableOption> selectables;

    public GameObject sliderPrefab;
    public GameObject togglePrefab;
    public GameObject adPromptPrefab;

    public Transform contentHolder;

    public Vector2 topSelectableCoordinates;

    public float yChange;

    public bool saveToPlayerPrefs = true;

    public bool autoLoadDefaults;

    [ExecuteInEditMode]
    private void OnValidate()
    {
        SetIDs();
    }

    private GameObject PrefabFromSelectableType(SelectableOptionsStruct.Type type)
    {
        switch (type)
        {
            case SelectableOptionsStruct.Type.Slider: return sliderPrefab;
            case SelectableOptionsStruct.Type.Toggle: return togglePrefab;
            default: return null;
        }
    }

    public virtual void Start()
    {
        Vector2 curSelectableCoordinates = topSelectableCoordinates;

        foreach (SelectableOptionsStruct selectableStruct in selectableStructs)
        {
            GameObject selectableGameObject = Instantiate(PrefabFromSelectableType(selectableStruct.type), contentHolder);

            SelectableOption selectableOption = selectableGameObject.GetComponentInChildren<SelectableOption>();
            selectableOption.Setup(selectableStruct);

            selectableGameObject.GetComponent<RectTransform>().anchoredPosition = curSelectableCoordinates;

            curSelectableCoordinates.y -= yChange;

            if (selectableOption.options.ad)
            {
                GameObject adPrompt = Instantiate(adPromptPrefab, selectableGameObject.transform);
                selectableOption.SetupAdPrompt(adPrompt);
            }

            selectables.Add(selectableOption);
        }

        if (autoLoadDefaults)
        {
            LoadSettings();
        }
    }

    private void SetIDs()
    {
        foreach (SelectableOptionsStruct selectableStruct in selectableStructs)
        {
            selectableStruct.GenerateID();
        }
    }

    public void LoadSettings()
    {
        foreach (SelectableOption option in selectables)
        {
            if (PlayerPrefs.HasKey(option.id))
            {
                if (option.saveAsInt)
                {
                    option.SetValue(PlayerPrefs.GetInt(option.id, 0));
                }
                else
                {
                    option.SetValue(PlayerPrefs.GetFloat(option.id, 0));
                }
            }
        }
    }

    public void SetDefaultSettings()
    {
        foreach (SelectableOption option in selectables)
        {
            option.SetDefault();
        }
    }

    public virtual void SaveOptions()
    {
        if (saveToPlayerPrefs)
        {
            foreach (SelectableOption selectable in selectables)
            {
                if (selectable.saveAsInt)
                {
                    PlayerPrefs.SetInt(selectable.id, (int)selectable.Value());
                }
                else
                {
                    PlayerPrefs.SetFloat(selectable.id, selectable.Value());
                }
            }

            GameDataHolder initialSettings = new GameDataHolder();
            initialSettings.LoadSettingsFromPlayerPrefs();

            LoadSaveManager.Instance.SaveInitialSimulationSettings(initialSettings);

            PlayerPrefs.Save();
        }
    }
}
