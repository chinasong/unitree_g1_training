import type {ReactNode} from 'react';
import {CaretDownIcon} from '@phosphor-icons/react';

type Props = {
 title:string;
 summary:string;
 children:ReactNode;
 className?:string;
};

export default function Disclosure({title,summary,children,className=''}:Props){
 return <details className={`disclosure ${className}`.trim()}>
  <summary><span><b>{title}</b><small>{summary}</small></span><CaretDownIcon size={17}/></summary>
  <div className="disclosure-content">{children}</div>
 </details>;
}
