#include "GenerateGraphDialog.h"

GenerateGraphDialog::GenerateGraphDialog()
{
    m_comboBox = new QComboBox();
    m_comboBox->addItems(QStringList()<<"Graph"<<"Web"<<"Tree");



   m_vertexCountLabel = new QLabel("Count of Vertexes: ");
   m_vertexCountSB = new QSpinBox();
   m_parametrsOfGraphGB = new QGroupBox("Parametrs: ");
   m_generateButton = new QPushButton("Generate");
   m_cancelButton = new QPushButton("Cancel");

   int j = 1;
   m_mainLayout = new QGridLayout;
   m_mainLayout->addWidget(m_comboBox);
   m_mainLayout->addWidget(m_vertexCountLabel,j,0);
   m_mainLayout->addWidget(m_vertexCountSB,j,1);

   setLayout(m_mainLayout);

   radio1 = new QCheckBox("Weighted",m_parametrsOfGraphGB);
   radio2 = new QCheckBox("Random",m_parametrsOfGraphGB);
   radio3 = new QCheckBox("Oriented",m_parametrsOfGraphGB);
   radio4 = new QCheckBox("Acyclic");
   radio5 = new QCheckBox("Linked");

   GBl = new QVBoxLayout();
   GBl->addWidget(radio1);
   GBl->addWidget(radio2);
   GBl->addWidget(radio3);
   m_parametrsOfGraphGB->setLayout(GBl);

   m_graphLayout = new QGridLayout();
   prepareGraphMenu();

   j = 0;
   m_webLayout = new QGridLayout();
   prepareWebMenu();
   m_webMenuParent = new QWidget();
   m_webMenuParent->setLayout(m_webLayout);




   m_flags.insert("weighted",0);
   m_flags.insert("random",0);
   m_flags.insert("vertex_count",0);
   m_flags.insert("acyclic",0);
   m_flags.insert("linked",0);
   m_flags.insert("oriented",0);
   m_flags.insert("graph",0);
   m_flags.insert("web",0);
   m_flags.insert("tree",0);

   connect(m_generateButton, SIGNAL(clicked()), this, SLOT(s_checkData()));
   connect(radio2,SIGNAL(clicked(bool)),this,SLOT(s_showForRandom(bool)));
   connect(radio3,SIGNAL(clicked(bool)),this,SLOT(s_showForOriented(bool)));
   connect(m_comboBox,SIGNAL(currentTextChanged(QString)),this,SLOT(s_showBySelected(QString)));
   connect(m_cancelButton, &QAbstractButton::clicked, this, &QDialog::reject);

   s_showBySelected("Graph");

}

const QMap<QString, int> &GenerateGraphDialog::flags() const
{
    return m_flags;
}

void GenerateGraphDialog::prepareGraphMenu()
{
    m_graphLayout->setColumnStretch(1,2);
    int j = 1;
 //   m_graphLayout->addWidget(m_vertexCountLabel,j++,0);
 //   m_graphLayout->addWidget(m_vertexCountSB,j-1,1);

    m_graphLayout->addWidget(m_parametrsOfGraphGB,j,0,j,2);

    m_graphLayout->addWidget(m_generateButton,++j,0);
    m_graphLayout->addWidget(m_cancelButton,j,1);

    m_graphMenuParent = new QWidget();
    m_graphMenuParent->setLayout(m_graphLayout);
}

void GenerateGraphDialog::prepareWebMenu()
{
    int j = 0;
    m_webLayout->setColumnStretch(1,2);
    m_webLayout->addWidget(m_generateButton,++j,0);
    m_webLayout->addWidget(m_cancelButton,j,1);

}

void GenerateGraphDialog::s_checkData()
{
    //radio1 -> weighted
    if(radio1->isChecked()){
        m_flags["weighted"] = 1;
    }

    //radio2 -> random generation
    if(radio2->isChecked()){
        m_flags["random"] = 1;
    }

    //now-> vertex_count
    m_flags["vertex_count"] = m_vertexCountSB->value();

    //radio3 -> oriented
    if(radio3->isChecked()){
        m_flags["oriented"] = 1;
    }

    //radio4 -> acyclic
    if(radio4->isChecked()){
        m_flags["acyclic"] = 1;
    }

    //radio5 -> linked
    if(radio5->isChecked()){
        m_flags["linked"] = 1;
    }

    m_flags[m_comboBox->currentText().toLower()]=1;

    if(m_flags["web"]==1){
        m_flags["oriented"] = 1;
        m_flags["acyclic"] = 1;
        m_flags["linked"] = 1;
        m_flags["random"] = 1;
        m_flags["weighted"] = 1;
    }

    this->accept();
}

void GenerateGraphDialog::s_showForRandom(bool checked)
{
    if(checked){
        GBl->addWidget(radio5);
    }else{
        radio5->setParent(nullptr);
        radio5->setChecked(false);
        GBl->removeWidget(radio5);
    }
}

void GenerateGraphDialog::s_showForOriented(bool checked)
{
    if(checked){
        GBl->addWidget(radio4);
    }else{
        radio4->setParent(nullptr);
        radio4->setChecked(false);
        GBl->removeWidget(radio4);
    }
}

void GenerateGraphDialog::s_showBySelected(QString option_title)
{

    if(option_title=="Graph"){
        prepareGraphMenu();
        m_mainLayout->addWidget(m_graphMenuParent,2,0,2,2);
    }else if(m_previousOption=="Graph"){
        m_graphMenuParent->setParent(nullptr);
        m_mainLayout->removeWidget(m_graphMenuParent);
    }


    if(option_title=="Web"){
        prepareWebMenu();
        m_mainLayout->addWidget(m_webMenuParent,2,0,2,2);
    }/*else if(m_previousOption=="Web"){
        m_graphMenuParent->setParent(nullptr);
        m_mainLayout->removeWidget(m_webMenuParent);
    }*/

    m_previousOption=option_title;



}

GenerateGraphInput::GenerateGraphInput()
{
    int j = 1;
    m_mainLayout = new QGridLayout;
    m_textEdit = new QTextEdit();
    m_mainLayout->addWidget(m_textEdit,++j,0);
    m_generateButton = new QPushButton("Generate");
    m_cancelButton = new QPushButton("Cancel");
    m_mainLayout->addWidget(m_generateButton,++j,0);
    //m_mainLayout->addWidget(m_cancelButton,j,0);

    setLayout(m_mainLayout);

    connect(m_generateButton, SIGNAL(clicked()), this, SLOT(s_checkData()));
}

void GenerateGraphInput::s_checkData()
{
    qDebug()<<m_textEdit->toPlainText();
    QString str = m_textEdit->toPlainText();
    std::vector<std::vector<int>> result;

    // Разбиваем строку на строки по символу новой строки
    QStringList lines = str.split("\n", QString::SkipEmptyParts);
    for (const QString &line : lines) {
        std::vector<int> row;

        // Разбиваем каждую строку на числа по пробелу
        QStringList numbers = line.split(" ", QString::SkipEmptyParts);
        for (const QString &number : numbers) {
            row.push_back(number.toInt());
        }

        result.push_back(row);
    }
    adjMatrix=result;
    this->accept();
}
