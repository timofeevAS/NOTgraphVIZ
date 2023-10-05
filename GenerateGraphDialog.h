#ifndef GENERATEGRAPHDIALOG_H
#define GENERATEGRAPHDIALOG_H

#include <QObject>
#include <QtWidgets>
#include <QMap>

class GenerateGraphDialog : public QDialog
{
    Q_OBJECT
public:

    GenerateGraphDialog();


    const QMap<QString, int> &flags() const;

private:
    QSpinBox* m_vertexCountSB;
    QGroupBox* m_parametrsOfGraphGB;
    QComboBox* m_comboBox;
    QGridLayout* m_mainLayout;

    QGridLayout* m_graphLayout;
    QGridLayout* m_webLayout;
    QWidget* m_graphMenuParent;
    QWidget* m_webMenuParent;

    QCheckBox* radio1,*radio2,*radio3,*radio4,*radio5;
    QVBoxLayout* GBl; //layout for checkboxes "ratioI"

    QLabel* m_vertexCountLabel;
    QPushButton* m_generateButton;
    QPushButton* m_cancelButton;
    QMap<QString,int>m_flags;

    QString m_previousOption;

    void prepareGraphMenu();
    void prepareWebMenu();
private slots:
    void s_checkData();
    void s_showForRandom(bool checked);
    void s_showForOriented(bool checked);
    void s_showBySelected(QString option_title);
};


class GenerateGraphInput : public QDialog
{
    Q_OBJECT
public:
    GenerateGraphInput();
    std::vector<std::vector<int>> adjMatrix;
private:
    QSpinBox* m_vertexCountSB;
    QTextEdit* m_textEdit;
    QGridLayout* m_mainLayout;
    QPushButton* m_generateButton;
    QPushButton* m_cancelButton;

private slots:
    void s_checkData();
};

#endif // GENERATEGRAPHDIALOG_H
