"""比較レポート用の日本語PDF・HTML・編集可能なWord出力。"""
import base64
from html import escape
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from xml.etree import ElementTree as E
from PIL import Image as PILImage
from fontTools.ttLib import TTFont as FontFile
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak


class LocalReport:
    def __init__(self,title):
        self.title=title;self.blocks=[]

    def page(self,title):self.blocks.append(('page',title))
    def text(self,text,small=False):self.blocks.append(('small' if small else 'text',text))
    def table(self,rows,widths):self.blocks.append(('table',(rows,widths)))
    def image(self,path,width=510):self.blocks.append(('image',(Path(path),width)))

    def write(self,prefix):
        prefix=Path(prefix)
        jp='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
        latin='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        pdfmetrics.registerFont(TTFont('ReportJP',jp));pdfmetrics.registerFont(TTFont('ReportLatin',latin))
        jpchars=set(FontFile(jp).getBestCmap())
        def markup(text):
            chunks=[];current='';previous=None
            for char in str(text):
                font='ReportJP' if ord(char)>127 and ord(char) in jpchars else 'ReportLatin'
                if previous!=font and current:chunks.append(f'<font name="{previous}">{escape(current)}</font>');current=''
                previous=font;current+=char
            if current:chunks.append(f'<font name="{previous}">{escape(current)}</font>')
            return ''.join(chunks)
        styles={name:ParagraphStyle(name,fontName='ReportJP',fontSize=size,leading=leading,spaceAfter=after,
                    textColor=colors.HexColor('#24364A'),wordWrap='CJK')
                for name,size,leading,after in [('heading',17,25,12),('text',9.5,15,9),('small',8,12,6),('cell',8,12,0)]}
        flow=[];html=[];page_count=0
        for kind,value in self.blocks:
            if kind=='page':
                if page_count:flow.append(PageBreak());html.append('</section>')
                page_count+=1;html.append('<section>');flow.append(Paragraph(markup(value),styles['heading']));html.append(f'<h1>{escape(value)}</h1>')
            elif kind in ('text','small'):
                flow.append(Paragraph(markup(value),styles[kind]));html.append(f'<p class="{kind}">{escape(value)}</p>')
            elif kind=='table':
                rows,widths=value;table=Table([[Paragraph(markup(c),styles['cell']) for c in row] for row in rows],colWidths=widths,repeatRows=1,hAlign='LEFT')
                table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#E6EEF5')),('VALIGN',(0,0),(-1,-1),'TOP'),
                    ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
                    ('LINEBELOW',(0,0),(-1,0),.8,colors.HexColor('#91A7BC')),('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#DCE3EB'))]))
                flow.extend([table,Spacer(1,9)]);html.append('<table>'+''.join('<tr>'+''.join(f'<{"th" if i==0 else "td"}>{escape(str(c))}</{"th" if i==0 else "td"}>' for c in row)+'</tr>' for i,row in enumerate(rows))+'</table>')
            elif kind=='image':
                path,width=value;w,h=PILImage.open(path).size;flow.extend([Image(str(path),width=width,height=width*h/w),Spacer(1,7)])
                html.append(f'<img alt="{escape(path.stem)}" src="data:image/png;base64,{base64.b64encode(path.read_bytes()).decode()}">')
        html.append('</section>')
        def footer(canvas,doc):
            canvas.saveState();canvas.setStrokeColor(colors.HexColor('#DCE3EB'));canvas.line(42,35,553,35)
            p=Paragraph(markup(f'ミリ波・テラヘルツ通信の初回リンク確立　|　{doc.page}'),styles['small']);p.wrap(510,20);p.drawOn(canvas,42,18);canvas.restoreState()
        doc=SimpleDocTemplate(str(prefix.with_suffix('.pdf')),pagesize=A4,leftMargin=42,rightMargin=42,topMargin=40,bottomMargin=48,
                             title=self.title,author='通信シミュレーション比較')
        doc.build(flow,onFirstPage=footer,onLaterPages=footer)
        css='''body{font-family:"Noto Sans CJK JP","Yu Gothic",sans-serif;background:#eef2f6;color:#24364a;line-height:1.8;margin:0}main{max-width:1050px;margin:auto}section{background:white;padding:40px;margin:26px 0;border-radius:12px;box-shadow:0 4px 24px #20344a12}h1{font-size:25px;line-height:1.5}.small{font-size:13px;color:#536578}table{width:100%;border-collapse:collapse;font-size:14px;margin:16px 0}th{background:#e6eef5;text-align:left}td,th{padding:9px;border-bottom:1px solid #dce3eb;vertical-align:top}img{width:100%;height:auto}@media(max-width:700px){section{padding:18px}table{font-size:12px}}@media print{body{background:white}section{page-break-after:always;margin:0;padding:0;box-shadow:none}}'''
        prefix.with_suffix('.html').write_text(f'<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{escape(self.title)}</title><style>{css}</style><body><main>'+''.join(html)+'</main></body></html>')
        self._word(prefix.with_suffix('.docx'))

    def _word(self,path):
        ns={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main','r':'http://schemas.openxmlformats.org/officeDocument/2006/relationships',
            'wp':'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing','a':'http://schemas.openxmlformats.org/drawingml/2006/main',
            'pic':'http://schemas.openxmlformats.org/drawingml/2006/picture'}
        for key,url in ns.items():E.register_namespace(key,url)
        def tag(value):prefix,name=value.split(':');return '{'+ns[prefix]+'}'+name
        def child(parent,name,attrs=None,text=None):
            e=E.SubElement(parent,tag(name),{tag(k) if ':' in k else k:str(v) for k,v in (attrs or {}).items()})
            if text is not None:e.text=str(text)
            return e
        def paragraph(parent,text='',heading=False,small=False):
            p=child(parent,'w:p');pr=child(p,'w:pPr')
            if heading:child(pr,'w:keepNext')
            child(pr,'w:spacing',{'w:after':140,'w:line':285,'w:lineRule':'auto'})
            if heading:child(pr,'w:outlineLvl',{'w:val':0})
            r=child(p,'w:r');rp=child(r,'w:rPr');child(rp,'w:rFonts',{'w:ascii':'DejaVu Sans','w:hAnsi':'DejaVu Sans','w:eastAsia':'Noto Sans CJK JP'})
            if heading:child(rp,'w:b')
            child(rp,'w:color',{'w:val':'24364A'});child(rp,'w:sz',{'w:val':32 if heading else 16 if small else 19})
            child(r,'w:t',text=text)
            return p
        document=E.Element(tag('w:document'));body=child(document,'w:body');rels=[];images=[];pages=0
        for kind,value in self.blocks:
            if kind=='page':
                if pages:
                    p=child(body,'w:p');r=child(p,'w:r');child(r,'w:br',{'w:type':'page'})
                pages+=1;paragraph(body,value,heading=True)
            elif kind in ('text','small'):paragraph(body,value,small=kind=='small')
            elif kind=='table':
                rows,widths=value;table=child(body,'w:tbl');pr=child(table,'w:tblPr');child(pr,'w:tblW',{'w:w':10200,'w:type':'dxa'})
                borders=child(pr,'w:tblBorders')
                for side in ('top','bottom','insideH'):child(borders,'w:'+side,{'w:val':'single','w:sz':4,'w:color':'DCE3EB'})
                grid=child(table,'w:tblGrid')
                for width in widths:child(grid,'w:gridCol',{'w:w':int(width*20)})
                for i,row in enumerate(rows):
                    tr=child(table,'w:tr');trpr=child(tr,'w:trPr');child(trpr,'w:cantSplit')
                    if i==0:child(trpr,'w:tblHeader')
                    for text,width in zip(row,widths):
                        tc=child(tr,'w:tc');pr=child(tc,'w:tcPr');child(pr,'w:tcW',{'w:w':int(width*20),'w:type':'dxa'})
                        if i==0:child(pr,'w:shd',{'w:fill':'E6EEF5'})
                        paragraph(tc,str(text),small=True)
                paragraph(body,'',small=True)
            elif kind=='image':
                source,width=value;w,h=PILImage.open(source).size;cx=int(width*12700);cy=int(width*h/w*12700);number=len(images)+1;rid='rId'+str(number)
                images.append(source);rels.append((rid,f'media/figure{number}.png'))
                p=child(body,'w:p');r=child(p,'w:r');drawing=child(r,'w:drawing');inline=child(drawing,'wp:inline',{'distT':0,'distB':0,'distL':0,'distR':0})
                child(inline,'wp:extent',{'cx':cx,'cy':cy});child(inline,'wp:docPr',{'id':number,'name':source.stem})
                graphic=child(inline,'a:graphic');data=child(graphic,'a:graphicData',{'uri':ns['pic']});pic=child(data,'pic:pic')
                nv=child(pic,'pic:nvPicPr');child(nv,'pic:cNvPr',{'id':number,'name':source.name});child(nv,'pic:cNvPicPr')
                fill=child(pic,'pic:blipFill');child(fill,'a:blip',{'r:embed':rid});stretch=child(fill,'a:stretch');child(stretch,'a:fillRect')
                shape=child(pic,'pic:spPr');transform=child(shape,'a:xfrm');child(transform,'a:off',{'x':0,'y':0});child(transform,'a:ext',{'cx':cx,'cy':cy})
                geometry=child(shape,'a:prstGeom',{'prst':'rect'});child(geometry,'a:avLst')
        section=child(body,'w:sectPr');child(section,'w:pgSz',{'w:w':11906,'w:h':16838});child(section,'w:pgMar',{'w:top':800,'w:right':840,'w:bottom':960,'w:left':840,'w:header':360,'w:footer':360,'w:gutter':0})
        def xml(e):return E.tostring(e,encoding='utf-8',xml_declaration=True)
        relationships=E.Element('Relationships',xmlns='http://schemas.openxmlformats.org/package/2006/relationships')
        for rid,target in rels:E.SubElement(relationships,'Relationship',Id=rid,Type=ns['r']+'/image',Target=target)
        types=E.Element('Types',xmlns='http://schemas.openxmlformats.org/package/2006/content-types')
        for extension,content in [('rels','application/vnd.openxmlformats-package.relationships+xml'),('xml','application/xml'),('png','image/png')]:E.SubElement(types,'Default',Extension=extension,ContentType=content)
        E.SubElement(types,'Override',PartName='/word/document.xml',ContentType='application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml')
        top=E.Element('Relationships',xmlns='http://schemas.openxmlformats.org/package/2006/relationships');E.SubElement(top,'Relationship',Id='rId1',Type=ns['r']+'/officeDocument',Target='word/document.xml')
        with ZipFile(path,'w',ZIP_DEFLATED) as z:
            z.writestr('[Content_Types].xml',xml(types));z.writestr('_rels/.rels',xml(top));z.writestr('word/document.xml',xml(document));z.writestr('word/_rels/document.xml.rels',xml(relationships))
            for i,source in enumerate(images,1):z.write(source,f'word/media/figure{i}.png')
